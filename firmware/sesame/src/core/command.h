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
// Face expressions. This is a WIRE enum -- the companion app sends these
// by name over the control link, and the numeric values are what a future
// binary protocol or logged event would carry.
//
// APPEND ONLY. Never renumber an existing member: a stale companion, a
// saved animation, or a recorded log would silently mean a different
// face. New expressions go at the end, before Count.
//
// The last four exist for conversation rather than motion. A voice
// assistant needs to show that it heard you (Listening), that it is
// working (Thinking), that it failed to understand (Confused), and that
// it is replying (Talking). Without those the robot looks frozen during
// exactly the moments a person is waiting on it.
enum class FaceId : uint8_t {
  Neutral = 0,
  Happy = 1,
  Sad = 2,
  Angry = 3,
  Sleep = 4,
  Listening = 5,
  Thinking = 6,
  Confused = 7,
  Talking = 8,
  Count = 9,
};

struct DrivePayload {
  float vx;
  float omega;
  float bodyHeightMm;
  float stepHeightMm;
  // NOTE: no vy/strafe. A 2-DOF leg has no achievable lateral foot
  // velocity at neutral stance; off-neutral it flips sign across the
  // stride, which is a wobble rather than a strafe. Cut, not degraded --
  // see core/gait.h's GaitCommand comment. There is deliberately no
  // token position left in the `drive` grammar below for it either, so
  // a stale 5-argument line fails outright (wrong argument count) rather
  // than silently reinterpreting one of these fields as vy.
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
//   drive <vx> <omega> <bodyHeightMm> <stepHeightMm>   (no vy -- see DrivePayload)
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
