#include "control_server.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <stdio.h>

#include "../../config.h"
#include "../core/command.h"

// secrets.h is gitignored (this repo is public). Fall back to the example
// so a fresh clone compiles and runs on the robot's own access point
// straight away -- someone should be able to build this without first
// hunting for a file they were never told to create.
#if defined(__has_include)
#if __has_include("../../secrets.h")
#include "../../secrets.h"
#else
#include "../../secrets_example.h"
#endif
#else
#include "../../secrets_example.h"
#endif

namespace sesame {

namespace {

WebServer g_server(80);
Mailbox* g_mailbox = nullptr;

const char* kModeNames[] = {"hold", "stand", "rest", "manual", "drive",
                            "detached"};

const char kIndexHtml[] PROGMEM = R"HTML(<!doctype html><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Sesame</title><style>
body{font:16px system-ui;margin:0;padding:1rem;background:#111;color:#eee}
h1{font-size:1.1rem;margin:0 0 1rem}
button{font:inherit;padding:.9rem;margin:.2rem;border:0;border-radius:.5rem;
background:#2a2a2a;color:#eee;min-width:5.5rem}
button:active{background:#444}
.stop{background:#7a1f1f}
pre{background:#000;padding:.6rem;border-radius:.4rem;overflow:auto;font-size:12px}
</style><h1>Sesame</h1>
<div>
<button onclick="d(40,0)">forward</button>
<button onclick="d(-40,0)">back</button>
<button onclick="d(0,40)">left</button>
<button onclick="d(0,-40)">right</button>
</div><div>
<button onclick="c('stand')">stand</button>
<button onclick="c('rest')">rest</button>
<button class=stop onclick="c('stop')">stop</button>
<button class=stop onclick="fetch('/estop',{method:'POST'})">E-STOP</button>
</div>
<pre id=s>...</pre><script>
let t=null;
function c(l){return fetch('/cmd',{method:'POST',body:l}).then(r=>r.json())}
function d(vx,om){
  // Driving is a STREAM: the robot's watchdog stops it after 500ms of
  // silence, so the button must keep talking while it is held.
  clearInterval(t);
  const line='drive '+vx+' '+om+' 27.5 8.2';
  c(line); t=setInterval(()=>c(line),200);
  setTimeout(()=>{clearInterval(t)},10000);
}
setInterval(()=>fetch('/status').then(r=>r.text())
  .then(x=>document.getElementById('s').textContent=x),500);
</script>)HTML";

void sendJson(int code, const char* body) {
  g_server.send(code, "application/json", body);
}

void handleRoot() { g_server.send_P(200, "text/html", kIndexHtml); }

void handleStatus() {
  const RobotState s = g_mailbox->state();
  char buf[420];
  const char* mode =
      (s.mode < sizeof(kModeNames) / sizeof(kModeNames[0])) ? kModeNames[s.mode]
                                                            : "?";
  snprintf(buf, sizeof(buf),
           "{\"mode\":\"%s\",\"attached\":%s,\"estopped\":%s,"
           "\"throttle\":%.2f,\"vx\":%.1f,\"omega\":%.1f,"
           "\"battery\":%.2f,\"seq\":%lu,\"uptime_ms\":%lu,"
           "\"pose\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}",
           mode, s.attached ? "true" : "false", s.estopped ? "true" : "false",
           double(s.throttle), double(s.vxMmPerSec), double(s.omegaDegPerSec),
           double(s.batteryVolts), (unsigned long)s.seq,
           (unsigned long)s.uptimeMs, double(s.pose.deg[0]),
           double(s.pose.deg[1]), double(s.pose.deg[2]), double(s.pose.deg[3]),
           double(s.pose.deg[4]), double(s.pose.deg[5]), double(s.pose.deg[6]),
           double(s.pose.deg[7]));
  sendJson(200, buf);
}

void handleCmd() {
  if (g_server.method() != HTTP_POST) {
    sendJson(405, "{\"ok\":false,\"reason\":\"POST only\"}");
    return;
  }
  String body = g_server.arg("plain");
  body.trim();
  if (body.length() == 0 || body.length() >= core::kMaxCommandLineLen) {
    sendJson(400, "{\"ok\":false,\"reason\":\"empty or oversized\"}");
    return;
  }

  core::Command cmd;
  if (!core::parseCommand(body.c_str(), &cmd)) {
    // parseCommand already rejects malformed input, non-finite numbers,
    // and out-of-envelope calibration -- all covered by host tests.
    sendJson(400, "{\"ok\":false,\"reason\":\"parse\"}");
    return;
  }
  // seq is stamped by the mailbox -- one sequence space for all
  // transports. See Mailbox::postCommand.
  if (!g_mailbox->postCommand(cmd)) {
    // Queue full. Say so rather than dropping silently: a command the
    // caller believes was accepted but never ran is worse than an error.
    sendJson(503, "{\"ok\":false,\"reason\":\"busy\"}");
    return;
  }
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"ok\":true,\"seq\":%lu}",
           (unsigned long)g_mailbox->lastSeq());
  sendJson(200, buf);
}

void handleEstop() {
  // A separate endpoint so it can never queue behind a slow request.
  core::Command cmd;
  cmd.type = core::CmdType::EStop;
  g_mailbox->postCommand(cmd);
  sendJson(200, "{\"ok\":true,\"estopped\":true}");
}

void handleNotFound() { g_server.send(404, "text/plain", "not found"); }

// Owns the network stack entirely. Pinned to core 0 so a blocking socket
// read can never delay the 50Hz motion tick on core 1.
void ioTask(void*) {
  g_server.on("/", handleRoot);
  g_server.on("/status", handleStatus);
  g_server.on("/cmd", handleCmd);
  g_server.on("/estop", handleEstop);
  g_server.onNotFound(handleNotFound);
  g_server.begin();
  for (;;) {
    g_server.handleClient();
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

}  // namespace

bool ControlServer::begin(Mailbox* mailbox) {
  g_mailbox = mailbox;

  // AP_STA so the robot is always reachable on its own network even when
  // joining yours fails -- otherwise a typo in the password bricks your
  // only control path.
  WiFi.mode(WIFI_AP_STA);
  const bool apOk = WiFi.softAP(kApSsid, kApPassword);

  if (kStationSsid[0] != '\0') {
    WiFi.begin(kStationSsid, kStationPassword);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; ++i) {
      delay(500);  // setup() only, before the motion loop starts
    }
    staConnected_ = (WiFi.status() == WL_CONNECTED);
  }

  if (staConnected_) {
    snprintf(ipText_, sizeof(ipText_), "%s", WiFi.localIP().toString().c_str());
  } else {
    snprintf(ipText_, sizeof(ipText_), "%s", WiFi.softAPIP().toString().c_str());
  }

  if (MDNS.begin(kHostname)) {
    MDNS.addService("http", "tcp", 80);
  }

  // Over-the-air updates. Highest quality-of-life-per-line in the whole
  // project: servo channels 0 and 1 sit on GPIO 15 and GPIO 2, which are
  // ESP32 boot strapping pins, so a USB upload may require physically
  // unplugging two servos every single time. OTA sidesteps that for
  // every flash after the first.
  //
  // Servos are DETACHED before the update starts. A half-written
  // firmware image with eight servos still holding torque is how you
  // cook a battery or strip a gear while nobody is watching.
  ArduinoOTA.setHostname(kHostname);
  ArduinoOTA.onStart([]() {
    core::Command cmd;
    cmd.type = core::CmdType::EStop;
    if (g_mailbox != nullptr) {
      g_mailbox->postCommand(cmd);
    }
    delay(200);  // let the motion loop act on it before flash writes begin
  });
  ArduinoOTA.begin();

  xTaskCreatePinnedToCore(ioTask, "io", 8192, nullptr, 1, nullptr, 0);
  return apOk;
}

}  // namespace sesame
