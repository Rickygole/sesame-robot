// mailbox.h -- the only channel between the network task and the motion loop.
//
// The motion loop runs on core 1 at a hard 50Hz. The web server runs on
// core 0 because a blocking socket read inside the motion tick would
// jitter every servo. That split is only safe if the two sides share
// nothing but fixed-size POD passed through here.
//
// Two directions, two mechanisms:
//   commands  network -> motion    FreeRTOS queue (naturally serializing)
//   state     motion  -> network   portMUX-guarded snapshot
//
// Rules that make this safe:
//   * No pointers, no String, no heap cross the boundary. Command and
//     RobotState are POD, copied by value.
//   * The critical section holds for a struct copy (~1us), never for
//     I/O, parsing, or anything that can block.
//   * The network side NEVER touches servos, gait state, or I2C. It
//     produces Commands; the motion loop decides what to do with them.
#pragma once

#include <stdint.h>

#include "../core/command.h"
#include "../core/pose.h"

namespace sesame {

// A snapshot of what the robot is doing, for /status. Fixed size, no
// pointers -- safe to copy under a spinlock.
struct RobotState {
  uint8_t mode = 0;            // mirrors the sketch's Mode enum
  uint8_t faceId = 0;
  bool attached = false;
  bool estopped = false;
  float throttle = 1.f;        // 1.0 = not power-limited
  float vxMmPerSec = 0.f;
  float omegaDegPerSec = 0.f;
  float batteryVolts = 0.f;
  uint32_t seq = 0;            // last accepted command
  uint32_t uptimeMs = 0;
  core::JointPose pose;
};

class Mailbox {
 public:
  // Creates the queue. Call once from setup(), before the network task
  // starts.
  bool begin(uint8_t depth = 4);

  // --- network side (core 0) ---
  // Non-blocking. Returns false if the queue is full, which the caller
  // must surface as a rejection rather than silently dropping: a command
  // the user believes was accepted but never ran is worse than an error.
  //
  // STAMPS THE SEQUENCE NUMBER ITSELF, overwriting whatever the caller
  // put there. Do not set cmd.seq before calling.
  //
  // The envelope rejects any command whose seq is not newer than the
  // last one it saw -- correct anti-replay behaviour, but only if there
  // is ONE sequence space. There were two: the serial CLI and the HTTP
  // server each kept a private counter starting at zero. Once the web UI
  // had streamed a few hundred commands, every serial command arrived
  // looking centuries old and was silently refused with stale_seq --
  // the robot ignored the keyboard entirely and sat in stand.
  //
  // Centralising it here means a third transport (OTA, BLE, a script)
  // cannot reintroduce the bug by forgetting to share a counter.
  bool postCommand(core::Command cmd);

  // Next sequence number, for callers that need to report it back to a
  // client. Does not consume one.
  uint32_t lastSeq() const;

  // Copies the latest state snapshot.
  RobotState state() const;

  // --- motion side (core 1) ---
  // Non-blocking. Returns false if no command is waiting.
  bool takeCommand(core::Command* out);

  // Publishes a new snapshot. Called once per motion tick.
  void publish(const RobotState& s);

 private:
  void* queue_ = nullptr;  // QueueHandle_t, kept opaque so this header
                           // does not drag FreeRTOS into every includer
};

}  // namespace sesame
