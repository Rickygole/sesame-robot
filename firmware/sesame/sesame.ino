// sesame.ino -- Sesame quadruped firmware, ESP32-WROOM-32.
//
// This file is deliberately thin: setup(), loop(), and the command
// dispatch that glues them together. All real logic lives under src/ --
// src/core/ is pure C++ with no Arduino dependency (and is unit-tested on
// the host via `make test` from the repo root), src/board/ is the
// hardware layer.
//
// Implemented: servo bring-up, calibration (persisted to NVS), pose
// hold, slew limiting, serial CLI, walking (crawl gait,
// forward/back/turn/arc) with a drive watchdog, a latching e-stop, an
// OLED face, and a WiFi control surface with an HTTP API.
//
// Sensors are OPTIONAL and detected at boot. With an MPU6050 fitted the
// body levels itself on uneven ground; with an HC-SR04 fitted the robot
// stops before walking into things. Neither is required -- absent
// hardware is detected and the feature simply stays off.
//
// The IMU does NOT provide balance recovery, and cannot. With 2 DOF per
// leg there is no foot-position-preserving attitude control to be had,
// and the crawl gait is already statically stable. Levelling the body on
// a slope is the honest extent of it.
//
// NONE OF THIS HAS RUN ON HARDWARE. No board exists yet.
//
// Safety ordering, which is load-bearing: envelope clamps the TARGET,
// then slew limits the RATE. Two different jobs. Nothing bypasses either.
//
// There is deliberately no strafe. A 2-DOF leg's achievable foot motion
// at neutral stance is purely tangential to its hip circle, so lateral
// authority is exactly zero -- see GaitCommand in src/core/gait.h.
//
// Board settings: ESP32 Dev Module, Arduino Runs On = Core 1.

#include <Arduino.h>

#include "config.h"
#include "src/board/display.h"
#include "src/board/imu.h"
#include "src/board/rangefinder.h"
#include "src/board/servo_bank.h"
#include "src/board/storage.h"
#include "src/net/control_server.h"
#include "src/net/mailbox.h"
#include "src/core/calibration.h"
#include "src/core/command.h"
#include "src/core/envelope.h"
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

// Safety gate. Every Drive request passes through this BEFORE the gait
// sees it: envelope clamps the TARGET, slew limits the RATE. Two
// different jobs, both mandatory, in that order. Nothing bypasses either.
Envelope g_envelope;

sesame::Display g_display;
sesame::Imu g_imu;
sesame::Rangefinder g_range;
// Levelling authority. Deliberately well under 1.0 -- a full
// correction fights the gait and oscillates.
constexpr float kLevelGain = 0.35f;
float g_levelBias[kLegCount] = {0.f, 0.f, 0.f, 0.f};
sesame::Storage g_storage;
sesame::Mailbox g_mailbox;
sesame::ControlServer g_server;

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
  if (g_imu.present()) {
    Serial.print(F(" tilt="));
    Serial.print(g_imu.attitude().rollDeg, 1);
    Serial.print('/');
    Serial.print(g_imu.attitude().pitchDeg, 1);
  }
  {
    const uint16_t r = g_range.distanceMm();
    Serial.print(F(" range="));
    if (r == sesame::kRangeInvalidMm) {
      Serial.print(F("none"));  // UNKNOWN, not "clear"
    } else {
      Serial.print(r);
      Serial.print(F("mm"));
    }
  }
  Serial.print(F(" pose="));
  const JointPose& p = g_bank.lastWritten();
  for (uint8_t i = 0; i < kJointCount; ++i) {
    Serial.print(p.deg[i], 1);
    if (i + 1 < kJointCount) Serial.print(',');
  }
  Serial.println();
}

void handleCommand(const Command& cmd) {
  // While latched, refuse anything that could move the robot. Only an
  // explicit stand/rest clears it -- see CmdType::EStop below.
  if (g_envelope.estopped() && cmd.type != CmdType::Stand && cmd.type != CmdType::Rest &&
      cmd.type != CmdType::EStop && cmd.type != CmdType::Detach) {
    Serial.println(F("err estopped -- send 'stand' or 'rest' to clear"));
    return;
  }

  switch (cmd.type) {
    case CmdType::Stand:
      g_desired = standPose();
      g_mode = Mode::Stand;
      g_envelope.clearEstop();
      if (!g_bank.attached()) g_bank.attachAll();
      g_display.setFace(sesame::Face::Neutral);
      Serial.println(F("ok stand"));
      break;
    case CmdType::Rest:
      g_desired = restPose();
      g_mode = Mode::Rest;
      g_envelope.clearEstop();
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
      //
      // LATCHES. Once tripped, motion commands are refused until an
      // explicit `stand` or `rest` clears it. An e-stop you can undo by
      // accident is not an e-stop.
      g_bank.detachAll();
      g_mode = Mode::Detached;
      g_envelope.triggerEstop();
      g_display.setFace(sesame::Face::Angry);
      Serial.println(F("ok estop -- servos detached, LATCHED"));
      Serial.println(F("send 'stand' or 'rest' to clear"));
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
      if (g_storage.save(g_cal)) {
        Serial.println(F("ok savecal -- persisted, survives power cycle"));
      } else {
        // Say so loudly. A calibration you believe is saved and is not
        // costs a full recalibration at the next power-on.
        Serial.println(F("err savecal FAILED -- calibration NOT persisted"));
      }
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

      const EnvelopeResult env = g_envelope.apply(
          gc, cmd.seq, float(millis()) * 0.001f, ObstacleState::Clear,
          g_geo[0], g_gait.gait());
      if (!env.accepted) {
        Serial.print(F("err drive rejected: "));
        Serial.println(rejectReasonName(env.reason));
        break;
      }

      g_gait.setCommand(env.clamped);
      if (!g_bank.attached()) g_bank.attachAll();
      g_mode = Mode::Drive;
      g_lastDriveMs = millis();

      // Report ACHIEVED, not requested. If the envelope modified the
      // command, say so -- silently clamping is how a knob starts lying.
      Serial.print(F("ok drive vx="));
      Serial.print(env.clamped.vxMmPerSec, 1);
      Serial.print(F(" omega="));
      Serial.print(env.clamped.omegaDegPerSec, 1);
      if (env.wasClamped) {
        Serial.print(F("  (CLAMPED from "));
        Serial.print(d.vx, 1);
        Serial.print(' ');
        Serial.print(d.omega, 1);
        Serial.print(')');
      }
      Serial.println();
      break;
    }
    case CmdType::PlayClip:
      Serial.println(F("err playclip not implemented (stage 2)"));
      break;
    case CmdType::SetFace: {
      const uint8_t id = cmd.payload.setFace.faceId;
      if (id >= uint8_t(sesame::Face::Count)) {
        Serial.println(F("err face id"));
        break;
      }
      g_display.setFace(sesame::Face(id));
      Serial.print(F("ok setface "));
      Serial.println(faceName(FaceId(id)));
      break;
    }
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
          // Route through the mailbox rather than calling handleCommand
          // directly, so serial shares ONE sequence space with HTTP.
          // Previously each kept a private counter starting at zero; once
          // the web UI had streamed a few hundred commands, every serial
          // command looked stale and the envelope silently refused it.
          if (!g_mailbox.postCommand(cmd)) {
            Serial.println(F("err busy -- command queue full"));
          }
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

  if (g_display.begin()) {
    g_display.setFace(sesame::Face::Sleep);
    for (uint8_t i = 0; i < 8; ++i) g_display.flushChunk();  // setup() only
  } else {
    // Not fatal -- the robot walks fine without a face. Usually SDA/SCL
    // swapped, or the panel answers at 0x3D rather than 0x3C.
    Serial.println(F("warn: no SSD1306 at 0x3C -- check I2C wiring"));
  }

  applyDefaults(&g_cal);
  // A stored calibration replaces the defaults only if magic, version
  // and CRC all check out; otherwise the defaults stand.
  if (g_storage.begin() && g_storage.load(&g_cal)) {
    Serial.println(F("calibration loaded from flash"));
  } else {
    Serial.println(F("no stored calibration -- using defaults"));
  }
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
    // Print EVERY channel, not just the first bad one. One bad channel
    // and five bad channels have completely different causes.
    int32_t want[kJointCount];
    int32_t got[kJointCount];
    g_bank.selfTestReport(want, got);
    for (uint8_t i = 0; i < kJointCount; ++i) {
      Serial.print(F("  ch"));
      Serial.print(i);
      Serial.print(F(" gpio"));
      Serial.print(sesame::kServoPins[i]);
      Serial.print(F(" want="));
      Serial.print(want[i]);
      Serial.print(F(" got="));
      Serial.print(got[i]);
      Serial.println(got[i] == want[i] ? F("  ok") : F("  <-- MISMATCH"));
    }
    Serial.println(F("Expected vendored ESP32Servo 3.0.9 at src/vendor/."));
  } else {
    Serial.println(F("servo self-test ok"));
  }

  // Servos are at unknown physical positions at power-on, so the first
  // move is necessarily a large step on all eight. The slew limiter
  // cannot help (no known starting pose), so sequence them instead.
  g_desired = standPose();
  g_bank.homeSequenced(g_desired, 100);
  g_mode = Mode::Stand;
  g_display.setFace(sesame::Face::Neutral);

  // Sensors. Both are optional -- absent hardware is detected and the
  // feature stays off rather than faulting.
  if (g_imu.begin()) {
    // Assumes the robot is standing level on a flat surface right now,
    // which it is: homeSequenced() just put it there. This cancels
    // mounting misalignment and gyro bias in one step.
    g_imu.calibrateLevel();
    Serial.println(F("IMU found -- body levelling active"));
  } else {
    Serial.println(F("no IMU -- levelling off (this is fine)"));
  }
  if (g_range.begin()) {
    Serial.print(F("rangefinder on GPIO "));
    Serial.print(sesame::kRangeTrigPin);
    Serial.print('/');
    Serial.print(sesame::kRangeEchoPin);
    Serial.println(F(" -- ECHO MUST be level-shifted to 3.3V"));
  }

  // Network last: the robot must be mechanically safe before it starts
  // accepting commands from anywhere.
  g_mailbox.begin();
  if (g_server.begin(&g_mailbox)) {
    Serial.print(F("control surface at http://"));
    Serial.println(g_server.ipText());
    if (g_server.connectedToNetwork()) {
      Serial.println(F("joined your WiFi -- also on http://sesame.local"));
    } else {
      Serial.println(F("AP only -- connect to the 'Sesame' network"));
    }
    g_display.setStatus(g_server.ipText());
  } else {
    Serial.println(F("warn: access point failed to start"));
  }

  Serial.print(F("tick "));
  Serial.print(sesame::kTickMs);
  Serial.println(F(" ms. type 'help'."));
}

void loop() {
  static TickType_t last = xTaskGetTickCount();

  pollSerial();

  // Commands from the network arrive here, having been parsed on core 0
  // by the SAME parseCommand() the serial CLI uses. One command grammar,
  // one set of tests, two transports.
  Command netCmd;
  while (g_mailbox.takeCommand(&netCmd)) {
    handleCommand(netCmd);
  }

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

  // --- Sensors ------------------------------------------------------
  g_imu.update(dt);
  g_range.tick(dt);

  // Obstacle stop. An INVALID reading means UNKNOWN, not clear -- a
  // sensor that has stopped answering is not evidence of open space, so
  // it must not be treated as permission to keep walking. But nor should
  // a missing sensor stop the robot forever, which is why distanceMm()
  // returns invalid rather than zero when nothing is fitted.
  const uint16_t rangeMm = g_range.distanceMm();
  const bool blocked =
      (rangeMm != sesame::kRangeInvalidMm) && (rangeMm < sesame::kObstacleStopMm);
  if (blocked && g_mode == Mode::Drive) {
    g_desired = standPose();
    g_mode = Mode::Stand;
    g_display.setFace(sesame::Face::Confused);
    Serial.print(F("obstacle at "));
    Serial.print(rangeMm);
    Serial.println(F("mm -- stopping"));
  }

  // Body levelling. Only useful while standing or walking; pointless at
  // rest and actively wrong while a joint is being trimmed by hand.
  const bool levelling =
      g_imu.present() && (g_mode == Mode::Stand || g_mode == Mode::Drive);
  if (levelling) {
    const sesame::Attitude& att = g_imu.attitude();
    if (att.valid && fabsf(att.rollDeg) < sesame::kMaxRecoverableTiltDeg &&
        fabsf(att.pitchDeg) < sesame::kMaxRecoverableTiltDeg) {
      sesame::levelingBias(att.rollDeg, att.pitchDeg, sesame::kHalfBodyLenMm,
                           sesame::kHalfBodyWidMm, kLevelGain, g_levelBias);
    } else {
      // Past recoverable tilt the robot is falling or has been picked
      // up. Flailing a correction that cannot succeed makes it worse.
      for (uint8_t i = 0; i < kLegCount; ++i) g_levelBias[i] = 0.f;
    }
  } else {
    for (uint8_t i = 0; i < kLegCount; ++i) g_levelBias[i] = 0.f;
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

  // Apply the levelling correction as a knee delta. A stance-depth
  // change of dz needs dphi = dz / (leg * cos(phi)); at the nominal
  // stance that denominator is just the stance radius minus the coxa.
  if (levelling) {
    const float lever = g_geo[0].legMm * cosf(0.5236f);  // ~30deg nominal
    if (lever > 1.f) {
      for (uint8_t i = 0; i < kLegCount; ++i) {
        const float dDeg = (g_levelBias[i] / lever) * 57.29578f;
        g_desired.deg[logicalIndex(Leg(i), Joint::Knee)] += dDeg;
      }
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

  // ONE OLED page per tick (~2.9ms). A full Adafruit display() call
  // pushes 1024 bytes in one go, roughly 23ms -- it would blow the
  // entire 20ms budget and jitter every servo update behind the screen.
  g_display.tick(dt);
  g_display.flushChunk();

  // Publish a snapshot for /status. Fixed-size POD copied under a
  // spinlock held for about a microsecond.
  {
    sesame::RobotState st;
    st.mode = uint8_t(g_mode);
    st.faceId = uint8_t(g_display.face());
    st.attached = g_bank.attached();
    st.estopped = g_envelope.estopped();
    st.throttle = g_throttle;
    st.seq = g_mailbox.lastSeq();
    st.uptimeMs = millis();
    st.pose = g_bank.lastWritten();
    g_mailbox.publish(st);
  }

  // Fixed-rate tick. vTaskDelayUntil, not delay(), so jitter in the work
  // above does not accumulate into the period.
  vTaskDelayUntil(&last, pdMS_TO_TICKS(sesame::kTickMs));
}
