#include "storage.h"

#include <Preferences.h>

namespace sesame {

namespace {
Preferences g_prefs;
const char* kNamespace = "sesame";
const char* kCalKey = "cal";
}  // namespace

bool Storage::begin() {
  ready_ = g_prefs.begin(kNamespace, /*readOnly=*/false);
  return ready_;
}

bool Storage::load(core::Calibration* out) {
  if (!ready_ || out == nullptr) {
    return false;
  }
  // Read into a scratch copy, not into *out. If validation fails the
  // caller must keep its defaults untouched -- a partially-overwritten
  // calibration is worse than none.
  core::Calibration scratch;
  const size_t want = sizeof(scratch);
  const size_t have = g_prefs.getBytesLength(kCalKey);
  if (have != want) {
    return false;  // absent, or written by a different struct layout
  }
  const size_t got = g_prefs.getBytes(kCalKey, &scratch, want);
  if (got != want) {
    return false;
  }
  if (!scratch.isValid()) {
    // Bad magic, wrong version, or failed CRC. Discard rather than
    // repair: driving servos from a half-trusted calibration is how you
    // strip a gear during boot.
    return false;
  }
  *out = scratch;
  return true;
}

bool Storage::save(const core::Calibration& cal) {
  if (!ready_) {
    return false;
  }
  core::Calibration copy = cal;
  copy.magic = core::kCalibrationMagic;
  copy.version = core::kCalibrationVersion;
  copy.crc = copy.computeCrc();

  const size_t want = sizeof(copy);
  if (g_prefs.putBytes(kCalKey, &copy, want) != want) {
    return false;
  }

  // Read back and verify. A write that silently truncated would
  // otherwise be discovered only at the next boot, by which point the
  // calibration you thought you saved is gone.
  core::Calibration check;
  if (g_prefs.getBytes(kCalKey, &check, want) != want) {
    return false;
  }
  return check.isValid() && check.crc == copy.crc;
}

bool Storage::clear() {
  if (!ready_) {
    return false;
  }
  return g_prefs.remove(kCalKey);
}

}  // namespace sesame
