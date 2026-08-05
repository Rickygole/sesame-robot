// command.h -- POD command message + a host-testable CLI tokenizer.
//
// Command is fixed-size, no String, no heap: safe to pass over a byte
// pipe (WiFi, serial) unchanged and safe to unit test on a laptop.
#pragma once

#include <stdint.h>

namespace sesame {
namespace core {

enum class CmdType : uint8_t {
  Stop = 0,
  Stand = 1,
  Rest = 2,
  Drive = 3,
  PlayClip = 4,
  SetFace = 5,
  SetJoint = 6,
  SetCal = 7,
  SaveCal = 8,
  Detach = 9,
  EStop = 10,
  Count = 11,
};

// Placeholder face-expression IDs. The real face_state module is a
// later stage; this is just the id space SetFace commands address.
enum class FaceId : uint8_t {
  Neutral = 0,
  Happy = 1,
  Sad = 2,
  Angry = 3,
  Sleep = 4,
  Count = 5,
};

struct DrivePayload {
  float vx;
  float vy;
  float omega;
  float bodyHeightMm;
  float stepHeightMm;
};

struct PlayClipPayload {
  uint16_t clipId;
};

struct SetFacePayload {
  uint8_t faceId;
};

struct SetJointPayload {
  uint8_t jointIndex;  // logical index, see types.h::logicalIndex
  float deg;
};

struct SetCalPayload {
  uint8_t jointIndex;  // logical index
  int16_t usMin;
  int16_t usMax;
  float trimDeg;
  uint8_t invert;  // 0 or 1
  float minDeg;
  float maxDeg;
};

struct Command {
  CmdType type = CmdType::Stop;
  uint32_t seq = 0;
  union Payload {
    DrivePayload drive;
    PlayClipPayload playClip;
    SetFacePayload setFace;
    SetJointPayload setJoint;
    SetCalPayload setCal;
  } payload;
};

// ---- name <-> id tables ----------------------------------------------

// Canonical lowercase command token, e.g. "drive", "setface", "estop".
const char* commandName(CmdType type);
// Case-sensitive lookup of the above. Returns false if unknown.
bool commandTypeFromName(const char* name, CmdType* out);

const char* faceName(FaceId id);
bool faceIdFromName(const char* name, FaceId* out);

// ---- tokenizer ---------------------------------------------------------
//
// Whitespace-separated ASCII line, first token is the command name
// (see commandName()), remaining tokens are its arguments:
//   stop
//   stand
//   rest
//   drive <vx> <vy> <omega> <bodyHeightMm> <stepHeightMm>
//   playclip <clipId>
//   setface <faceName>
//   setjoint <jointIndex> <deg>
//   setcal <jointIndex> <usMin> <usMax> <trimDeg> <invert 0|1> <minDeg> <maxDeg>
//   savecal
//   detach
//   estop
//
// Does not allocate. Rejects malformed input (unknown command, wrong
// argument count, unparseable numbers, out-of-range joint index) by
// returning false and leaving *out untouched. seq is always left at 0;
// callers that need sequencing set out->seq themselves after a
// successful parse.
constexpr uint32_t kMaxCommandLineLen = 128;

bool parseCommand(const char* line, Command* out);

}  // namespace core
}  // namespace sesame
