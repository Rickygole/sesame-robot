// storage.h -- calibration persistence in NVS flash.
//
// Without this, every power cycle loses your servo trims and the robot
// has to be recalibrated from scratch. The upstream firmware "solves"
// this by printing a C array for you to paste back into the source and
// reflash, which is a poor trade for a value you tune interactively.
//
// NVS, not SPIFFS/LittleFS: there is no filesystem to justify, and NVS
// survives an OTA update.
//
// The Calibration struct carries its own magic, version and CRC, all
// validated by Calibration::isValid(). A blob that fails validation is
// DISCARDED rather than repaired -- a half-trusted calibration would
// drive servos to arbitrary angles at boot, which is how you strip a
// gear before the robot has finished starting up.
#pragma once

#include <stdint.h>

#include "../core/calibration.h"

namespace sesame {

class Storage {
 public:
  // Opens the NVS namespace. Safe to call once in setup().
  bool begin();

  // Loads into *out. Returns false if nothing is stored, the blob is the
  // wrong size, or isValid() rejects it (bad magic, wrong version, or
  // failed CRC). On false, *out is left untouched -- the caller keeps
  // its defaults.
  bool load(core::Calibration* out);

  // Recomputes the CRC and writes. Returns false if the write failed or
  // read-back verification did not match.
  bool save(const core::Calibration& cal);

  // Erases the stored blob so the next boot uses defaults.
  bool clear();

  bool ready() const { return ready_; }

 private:
  bool ready_ = false;
};

}  // namespace sesame
