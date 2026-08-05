#include <cstdio>
#include <cstring>

#include "check.h"
#include "core/command.h"
#include "core/types.h"

int g_fails = 0;

using sesame::core::CmdType;
using sesame::core::Command;
using sesame::core::commandName;
using sesame::core::commandTypeFromName;
using sesame::core::FaceId;
using sesame::core::faceIdFromName;
using sesame::core::faceName;
using sesame::core::kMaxCommandLineLen;
using sesame::core::parseCommand;

static void testCommandNameRoundTrip() {
  for (uint8_t i = 0; i < uint8_t(CmdType::Count); ++i) {
    const CmdType t = CmdType(i);
    const char* name = commandName(t);
    CHECK(name[0] != '\0');
    CmdType back;
    CHECK(commandTypeFromName(name, &back));
    CHECK(back == t);
  }
  CmdType dummy;
  CHECK(!commandTypeFromName("not-a-command", &dummy));
  CHECK(!commandTypeFromName("", &dummy));
}

static void testFaceNameRoundTrip() {
  for (uint8_t i = 0; i < uint8_t(FaceId::Count); ++i) {
    const FaceId f = FaceId(i);
    const char* name = faceName(f);
    CHECK(name[0] != '\0');
    FaceId back;
    CHECK(faceIdFromName(name, &back));
    CHECK(back == f);
  }
  FaceId dummy;
  CHECK(!faceIdFromName("grumpy", &dummy));
}

static void testParseSimpleNoArgCommands() {
  const char* lines[] = {"stop", "stand", "rest", "savecal", "detach", "estop"};
  const CmdType expect[] = {CmdType::Stop, CmdType::Stand,   CmdType::Rest,
                             CmdType::SaveCal, CmdType::Detach, CmdType::EStop};
  for (int i = 0; i < 6; ++i) {
    Command cmd;
    CHECK(parseCommand(lines[i], &cmd));
    CHECK(cmd.type == expect[i]);
  }
}

static void testParseDrive() {
  Command cmd;
  CHECK(parseCommand("drive 1.5 0.3 60 20", &cmd));
  CHECK(cmd.type == CmdType::Drive);
  CHECK_NEAR(cmd.payload.drive.vx, 1.5, 1e-5);
  CHECK_NEAR(cmd.payload.drive.omega, 0.3, 1e-5);
  CHECK_NEAR(cmd.payload.drive.bodyHeightMm, 60.0, 1e-5);
  CHECK_NEAR(cmd.payload.drive.stepHeightMm, 20.0, 1e-5);
}

static void testParseSetJoint() {
  Command cmd;
  CHECK(parseCommand("setjoint 5 12.5", &cmd));
  CHECK(cmd.type == CmdType::SetJoint);
  CHECK(cmd.payload.setJoint.jointIndex == 5);
  CHECK_NEAR(cmd.payload.setJoint.deg, 12.5, 1e-5);
}

static void testParseSetFace() {
  Command cmd;
  CHECK(parseCommand("setface happy", &cmd));
  CHECK(cmd.type == CmdType::SetFace);
  CHECK(cmd.payload.setFace.faceId == uint8_t(FaceId::Happy));
}

static void testParseSetCal() {
  Command cmd;
  CHECK(parseCommand("setcal 2 700 2900 1.5 1 -60 90", &cmd));
  CHECK(cmd.type == CmdType::SetCal);
  CHECK(cmd.payload.setCal.jointIndex == 2);
  CHECK(cmd.payload.setCal.usMin == 700);
  CHECK(cmd.payload.setCal.usMax == 2900);
  CHECK_NEAR(cmd.payload.setCal.trimDeg, 1.5, 1e-5);
  CHECK(cmd.payload.setCal.invert == 1);
  CHECK_NEAR(cmd.payload.setCal.minDeg, -60.0, 1e-5);
  CHECK_NEAR(cmd.payload.setCal.maxDeg, 90.0, 1e-5);
}

static void testParsePlayClip() {
  Command cmd;
  CHECK(parseCommand("playclip 42", &cmd));
  CHECK(cmd.type == CmdType::PlayClip);
  CHECK(cmd.payload.playClip.clipId == 42);
}

static void testRejectsMalformed() {
  Command cmd;
  CHECK(!parseCommand("", &cmd));
  CHECK(!parseCommand("   ", &cmd));
  CHECK(!parseCommand("bogus", &cmd));
  CHECK(!parseCommand("drive 1 2", &cmd));              // too few args
  CHECK(!parseCommand("drive 1 2 3 4 5", &cmd));        // too many args (also: no vy slot)
  CHECK(!parseCommand("drive a b c d", &cmd));          // garbage numbers
  CHECK(!parseCommand("drive 1.5x 2 3 4", &cmd));       // trailing garbage in a number
  CHECK(!parseCommand("setjoint 99 1.0", &cmd));        // out-of-range joint index
  CHECK(!parseCommand("setjoint -1 1.0", &cmd));        // negative -> out of range for uint8
  CHECK(!parseCommand("setface bogusface", &cmd));      // unknown face
  CHECK(!parseCommand("setcal 0 700 2900 1.5 2 -60 90", &cmd));  // invert must be 0/1
  CHECK(!parseCommand(nullptr, &cmd));
  CHECK(!parseCommand("stop", nullptr));
}

static void testRejectsOversizedLineWithoutOverflow() {
  char huge[kMaxCommandLineLen + 64];
  for (uint32_t i = 0; i < sizeof(huge) - 1; ++i) huge[i] = 'a';
  huge[sizeof(huge) - 1] = '\0';
  Command cmd;
  CHECK(!parseCommand(huge, &cmd));  // must reject cleanly, not overflow the internal buffer
}

static void testRejectsTooManyTokens() {
  Command cmd;
  CHECK(!parseCommand("drive 1 2 3 4 5 6 7 8 9 10 11 12", &cmd));
}

int main() {
  testCommandNameRoundTrip();
  testFaceNameRoundTrip();
  testParseSimpleNoArgCommands();
  testParseDrive();
  testParseSetJoint();
  testParseSetFace();
  testParseSetCal();
  testParsePlayClip();
  testRejectsMalformed();
  testRejectsOversizedLineWithoutOverflow();
  testRejectsTooManyTokens();
  if (g_fails) {
    std::fprintf(stderr, "%d failure(s)\n", g_fails);
  }
  return g_fails ? 1 : 0;
}
