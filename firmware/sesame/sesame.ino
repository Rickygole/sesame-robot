// sesame.ino -- Sesame quadruped firmware, ESP32-WROOM-32.
//
// This file is deliberately thin: setup(), loop(), and the command
// dispatch that glues them together. All real logic lives under src/ --
// src/core/ is pure C++ with no Arduino dependency (and is unit-tested on
// the host via `make test` from the repo root), src/board/ is the
// hardware layer.
//
// Implemented: servo bring-up, calibration, pose hold, slew limiting,
// serial CLI, and walking (crawl gait, forward/back/turn/arc) with a
// drive watchdog. NOT yet implemented: OLED face, WiFi, NVS-persisted
// calibration, IMU/ultrasonic.
//
// There is deliberately no strafe. A 2-DOF leg's achievable foot motion
// at neutral stance is purely tangential to its hip circle, so lateral
// authority is exactly zero -- see GaitCommand in src/core/gait.h.
//
// Board settings: ESP32 Dev Module, Arduino Runs On = Core 1.

#include <Arduino.h>

#include "config.h"
#include "src/board/servo_bank.h"
#include "src/core/calibration.h"
#include "src/core/command.h"
#include "src/core/gait.h"
#include "src/core/geometry.h"
#include "src/core/pose.h"
#include "src/core/slew.h"
#include "src/core/types.h"
#include "src/robot_config.h"

// The motion tick assumes loop() runs on core 1, leaving core 0 for the
// WiFi/network task added in a later stage. Tools > Arduino Runs On.
#if defined(CONFIG_ARDUINO_RUNNING_CORE) && CONFIG_ARDUINO_RUNNING_CORE != 1
#error "Set Tools > Arduino Runs On = Core 1."
#endif

using sesame::ServoBank;
using namespace sesame::core;

namespace {

ServoBank g_bank;
Calibration g_cal;
MotionLimits g_limits;

enum class Mode : uint8_t { Hold, Stand, Rest, Manual, Drive, Detached };
Mode g_mode = Mode::Stand;

JointPose g_desired;
uint32_t g_seq = 0;

GaitScheduler g_gait;
LegGeometry g_geo[kLegCount];
// Throttle from the previous tick, fed back into the gait clock so the
// gait stays coherent under power limiting rather than the pose
// distorting. This is the whole reason step() takes a throttle.
float g_throttle = 1.f;
uint32_t g_lastDriveMs = 0;

char g_line[kMaxCommandLineLen];
uint8_t g_lineLen = 0;

// Anatomical poses. Because calibration carries the per-joint trim and
// invert flags, these are symmetric across all four legs and readable --
// unlike the upstream firmware, where "stand" is the magic tuple
// 135/45/45/135, 180/0/0/180 with the mirroring baked into the numbers.
JointPose standPose() {
  JointPose p;
  for (uint8_t leg = 0; leg < kLegCount; ++leg) {
    p.deg[logicalIndex(Leg(leg), Joint::Hip)] = 0.f;
    p.deg[logicalIndex(Leg(leg), Joint::Knee)] = 55.f;
  }
  return p;
}

JointPose restPose() {
  JointPose p;
  for (uint8_t leg = 0; leg < kLegCount; ++leg) {
    p.deg[logicalIndex(Leg(leg), Joint::Hip)] = 0.f;
    p.deg[logicalIndex(Leg(leg), Joint::Knee)] = 0.f;
  }
  return p;
}

void applyDefaults(Calibration* cal) {
  for (uint8_t i = 0; i < kJointCount; ++i) {
    cal->joint[i].usMin = sesame::kServoUsMin;
    cal->joint[i].usMax = sesame::kServoUsMax;
    cal->joint[i].trimDeg = 0.f;
    cal->joint[i].invert = false;
    cal->joint[i].minDeg = -60.f;
    cal->joint[i].maxDeg = 90.f;
  }
  cal->magic = kCalibrationMagic;
  cal->version = kCalibrationVersion;
  cal->crc = cal->computeCrc();
}

void printHelp() {
  Serial.println(F("commands:"));
  Serial.println(F("  stand | rest | stop | estop | detach"));
  Serial.println(F("  setjoint <0-7> <deg>      logical index, see types.h"));
  Serial.println(F("  setcal <j> <usMin> <usMax> <trim> <inv 0|1> <min> <max>"));
  Serial.println(F("  drive <vx> <omega> <bodyH> <stepH>   mm/s, deg/s, mm, mm"));
  Serial.println(F("        no strafe: a 2-DOF leg has no lateral authority"));
  Serial.println(F("        resend within 500ms or it returns to stand"));
  Serial.println(F("  help | status"));
}

void printStatus() {
  Serial.print(F("mode="));
  switch (g_mode) {
    case Mode::Hold: Serial.print(F("hold")); break;
    case Mode::Stand: Serial.print(F("stand")); break;
    case Mode::Rest: Serial.print(F("rest")); break;
    case Mode::Manual: Serial.print(F("manual")); break;
    case Mode::Drive: Serial.print(F("drive")); break;
    case Mode::Detached: Serial.print(F("detached")); break;
  }
  Serial.print(F(" throttle="));
  Serial.print(g_throttle, 2);
  Serial.print(F(" attached="));
  Serial.print(g_bank.attached() ? 1 : 0);
  Serial.print(F(" pose="));
  const JointPose& p = g_bank.lastWritten();
  for (uint8_t i = 0; i < kJointCount; ++i) {
    Serial.print(p.deg[i], 1);
    if (i + 1 < kJointCount) Serial.print(',');
  }
  Serial.println();
}

void handleCommand(const Command& cmd) {
  switch (cmd.type) {
    case CmdType::Stand:
      g_desired = standPose();
      g_mode = Mode::Stand;
      if (!g_bank.attached()) g_bank.attachAll();
      Serial.println(F("ok stand"));
      break;
    case CmdType::Rest:
      g_desired = restPose();
      g_mode = Mode::Rest;
      if (!g_bank.attached()) g_bank.attachAll();
      Serial.println(F("ok rest"));
      break;
    case CmdType::Stop:
      g_desired = g_bank.lastWritten();
      g_mode = Mode::Hold;
      Serial.println(F("ok stop"));
      break;
    case CmdType::EStop:
      // Cut torque immediately. Holding a pose still draws current and
      // can still strip a gear if something is jammed; detaching is the
      // only true "stop".
      g_bank.detachAll();
      g_mode = Mode::Detached;
      Serial.println(F("ok estop -- servos detached"));
      break;
    case CmdType::Detach:
      g_bank.detachAll();
      g_mode = Mode::Detached;
      Serial.println(F("ok detach"));
      break;
    case CmdType::SetJoint: {
      const uint8_t j = cmd.payload.setJoint.jointIndex;
      if (j >= kJointCount) {
        Serial.println(F("err joint index"));
        break;
      }
      if (!g_bank.attached()) g_bank.attachAll();
      g_desired.deg[j] = cmd.payload.setJoint.deg;
      g_mode = Mode::Manual;
      Serial.print(F("ok setjoint "));
      Serial.print(j);
      Serial.print(' ');
      Serial.println(cmd.payload.setJoint.deg, 2);
      break;
    }
    case CmdType::SetCal: {
      const SetCalPayload& p = cmd.payload.setCal;
      JointCal& jc = g_cal.joint[p.jointIndex];
      jc.usMin = p.usMin;
      jc.usMax = p.usMax;
      jc.trimDeg = p.trimDeg;
      jc.invert = (p.invert != 0);
      jc.minDeg = p.minDeg;
      jc.maxDeg = p.maxDeg;
      g_cal.crc = g_cal.computeCrc();
      g_bank.setCalibration(g_cal);
      Serial.print(F("ok setcal "));
      Serial.println(p.jointIndex);
      break;
    }
    case CmdType::SaveCal:
      // NVS persistence is a later stage. Say so rather than silently
      // succeeding -- a calibration you think is saved and isn't costs a
      // full recalibration on the next power cycle.
      Serial.println(F("err savecal not implemented (stage 1b, NVS)"));
      break;
    case CmdType::Drive: {
      const DrivePayload& d = cmd.payload.drive;
      GaitCommand gc;
      gc.vxMmPerSec = d.vx;
      gc.omegaDegPerSec = d.omega;
      gc.bodyHeightMm = (d.bodyHeightMm > 0.f)
                            ? d.bodyHeightMm
                            : sesame::kDefaultBodyHeightFrac * g_geo[0].legMm;
      gc.stepHeightMm = (d.stepHeightMm > 0.f)
                            ? d.stepHeightMm
                            : sesame::kDefaultStepHeightFrac * g_geo[0].legMm;
      gc.maxStrideMm = 0.f;
      g_gait.setCommand(gc);
      if (!g_bank.attached()) g_bank.attachAll();
      g_mode = Mode::Drive;
      g_lastDriveMs = millis();
      Serial.print(F("ok drive vx="));
      Serial.print(d.vx, 1);
      Serial.print(F(" omega="));
      Serial.println(d.omega, 1);
      break;
    }
    case CmdType::PlayClip:
      Serial.println(F("err playclip not implemented (stage 2)"));
      break;
    case CmdType::SetFace:
      Serial.println(F("err setface not implemented (stage 1b, OLED)"));
      break;
    default:
      Serial.println(F("err unhandled"));
      break;
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = char(Serial.read());
    if (c == '\n' || c == '\r') {
      if (g_lineLen == 0) continue;
      g_line[g_lineLen] = '\0';
      if (strcmp(g_line, "help") == 0) {
        printHelp();
      } else if (strcmp(g_line, "status") == 0) {
        printStatus();
      } else {
        Command cmd;
        if (parseCommand(g_line, &cmd)) {
          cmd.seq = ++g_seq;
          handleCommand(cmd);
        } else {
          Serial.print(F("err parse: "));
          Serial.println(g_line);
        }
      }
      g_lineLen = 0;
    } else if (g_lineLen + 1 < kMaxCommandLineLen) {
      g_line[g_lineLen++] = c;
    } else {
      // Overlong line: drop it rather than truncating into a command that
      // happens to parse.
      g_lineLen = 0;
      Serial.println(F("err line too long"));
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(sesame::kSerialBaud);
  delay(200);
  Serial.println();
  Serial.println(F("sesame firmware -- stage 1a"));

  applyDefaults(&g_cal);
  g_bank.begin(g_cal);

  sesame::buildLegGeometry(g_geo);
  g_gait.setGeometry(g_geo);
  g_gait.setGait(kCrawl);  // duty 0.75 -- 3 feet down at all times

  uint8_t bad = 0;
  if (!g_bank.selfTest(&bad)) {
    // Almost always the wrong ESP32Servo (issue #103: writing one servo
    // disturbs another). Loud, because the symptom on the robot -- the
    // wrong leg moving -- looks exactly like a wiring fault.
    Serial.print(F("FATAL: servo self-test failed on wire channel "));
    Serial.println(bad);
    Serial.println(F("Expected vendored ESP32Servo 3.0.9 at src/vendor/."));
    Serial.println(F("Do NOT install ESP32Servo via Library Manager."));
  } else {
    Serial.println(F("servo self-test ok"));
  }

  // Servos are at unknown physical positions at power-on, so the first
  // move is necessarily a large step on all eight. The slew limiter
  // cannot help (no known starting pose), so sequence them instead.
  g_desired = standPose();
  g_bank.homeSequenced(g_desired, 100);
  g_mode = Mode::Stand;

  Serial.print(F("tick "));
  Serial.print(sesame::kTickMs);
  Serial.println(F(" ms. type 'help'."));
}

void loop() {
  static TickType_t last = xTaskGetTickCount();

  pollSerial();

  const float dt = float(sesame::kTickMs) * 0.001f;

  // Drive watchdog. If the controller stops talking mid-stride -- app
  // crashed, WiFi dropped, USB unplugged -- the robot must not keep
  // walking. Blend back to Stand rather than halting instantly, because
  // an instant switch is exactly the large step this design spends its
  // effort avoiding.
  if (g_mode == Mode::Drive &&
      (millis() - g_lastDriveMs) > sesame::kDriveWatchdogMs) {
    g_desired = standPose();
    g_mode = Mode::Stand;
    Serial.println(F("drive watchdog: no command in 500ms, standing"));
  }

  if (g_mode == Mode::Drive) {
    LegAngles ang[kLegCount];
    GaitReport rep;
    g_gait.step(dt, g_throttle, ang, rep);
    for (uint8_t i = 0; i < kLegCount; ++i) {
      g_desired.deg[logicalIndex(Leg(i), Joint::Hip)] = ang[i].hipDeg;
      g_desired.deg[logicalIndex(Leg(i), Joint::Knee)] = ang[i].kneeDeg;
    }
  }

  if (g_mode != Mode::Detached) {
    // applySlew is total: any non-finite input holds position rather than
    // propagating, so nothing here can produce an unsafe pulse width.
    const SlewResult r =
        applySlew(g_bank.lastWritten(), g_desired, g_limits, dt);
    g_bank.writePose(r.applied);
    g_throttle = r.throttle;
  }

  // Fixed-rate tick. vTaskDelayUntil, not delay(), so jitter in the work
  // above does not accumulate into the period.
  vTaskDelayUntil(&last, pdMS_TO_TICKS(sesame::kTickMs));
}
