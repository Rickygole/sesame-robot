// control_server.h -- WiFi + HTTP control surface, running on core 0.
//
// The wire format is deliberately the robot's EXISTING one-line CLI
// grammar, posted as text/plain:
//
//     POST /cmd     body: "drive 40 0 27.5 8.2"
//
// That reuses parseCommand() from src/core/command.cpp, which is already
// unit-tested on the host (firmware/host/test_command.cpp) and already
// rejects malformed input, non-finite numbers, and out-of-envelope
// calibration. No JSON parser on the device, no second grammar to keep
// in sync, no new bug surface -- and every command is curl-able and
// identical to what you would type into the serial console.
//
// Responses ARE JSON, hand-built. They are small and fixed-shape, so a
// JSON library would be a dependency bought for nothing.
//
// Responses report what was ACCEPTED, not merely that something was
// received. That is the same asked-vs-achieved discipline as GaitReport,
// carried onto the wire: it is what stops a remote command from becoming
// a knob that lies.
#pragma once

#include <stdint.h>

#include "mailbox.h"

namespace sesame {

class ControlServer {
 public:
  // Brings up the AP (always) and optionally joins your WiFi, starts
  // mDNS, and spawns the HTTP task PINNED TO CORE 0. The motion loop
  // owns core 1 and must never share it with a socket read.
  //
  // Returns false only if the AP could not start; station-mode failure
  // is non-fatal and reported via connectedToNetwork().
  bool begin(Mailbox* mailbox);

  bool connectedToNetwork() const { return staConnected_; }
  const char* ipText() const { return ipText_; }

 private:
  bool staConnected_ = false;
  char ipText_[32] = {0};
};

}  // namespace sesame
