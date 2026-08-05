#include "command.h"

#include <stdlib.h>
#include <string.h>

#include "types.h"

namespace sesame {
namespace core {

namespace {

struct NameEntry {
  const char* name;
};

constexpr NameEntry kCommandNames[uint8_t(CmdType::Count)] = {
    {"stop"}, {"stand"}, {"rest"},     {"drive"},  {"playclip"}, {"setface"},
    {"setjoint"}, {"setcal"}, {"savecal"}, {"detach"}, {"estop"},
};

constexpr NameEntry kFaceNames[uint8_t(FaceId::Count)] = {
    {"neutral"}, {"happy"}, {"sad"}, {"angry"}, {"sleep"},
};

// Parses a float token, rejecting empty/partial/garbage input.
bool parseFloat(const char* tok, float* out) {
  if (tok == nullptr || *tok == '\0') {
    return false;
  }
  char* end = nullptr;
  const float v = strtof(tok, &end);
  if (end == tok || *end != '\0') {
    return false;
  }
  *out = v;
  return true;
}

// Parses a base-10 integer token into a long, rejecting empty/partial/
// garbage input.
bool parseLong(const char* tok, long* out) {
  if (tok == nullptr || *tok == '\0') {
    return false;
  }
  char* end = nullptr;
  const long v = strtol(tok, &end, 10);
  if (end == tok || *end != '\0') {
    return false;
  }
  *out = v;
  return true;
}

bool parseU8(const char* tok, uint8_t* out) {
  long v = 0;
  if (!parseLong(tok, &v) || v < 0 || v > 255) {
    return false;
  }
  *out = uint8_t(v);
  return true;
}

bool parseI16(const char* tok, int16_t* out) {
  long v = 0;
  if (!parseLong(tok, &v) || v < -32768 || v > 32767) {
    return false;
  }
  *out = int16_t(v);
  return true;
}

bool parseU16(const char* tok, uint16_t* out) {
  long v = 0;
  if (!parseLong(tok, &v) || v < 0 || v > 65535) {
    return false;
  }
  *out = uint16_t(v);
  return true;
}

}  // namespace

const char* commandName(CmdType type) {
  const uint8_t idx = uint8_t(type);
  if (idx >= uint8_t(CmdType::Count)) {
    return "";
  }
  return kCommandNames[idx].name;
}

bool commandTypeFromName(const char* name, CmdType* out) {
  if (name == nullptr) {
    return false;
  }
  for (uint8_t i = 0; i < uint8_t(CmdType::Count); ++i) {
    if (strcmp(name, kCommandNames[i].name) == 0) {
      *out = CmdType(i);
      return true;
    }
  }
  return false;
}

const char* faceName(FaceId id) {
  const uint8_t idx = uint8_t(id);
  if (idx >= uint8_t(FaceId::Count)) {
    return "";
  }
  return kFaceNames[idx].name;
}

bool faceIdFromName(const char* name, FaceId* out) {
  if (name == nullptr) {
    return false;
  }
  for (uint8_t i = 0; i < uint8_t(FaceId::Count); ++i) {
    if (strcmp(name, kFaceNames[i].name) == 0) {
      *out = FaceId(i);
      return true;
    }
  }
  return false;
}

bool parseCommand(const char* line, Command* out) {
  if (line == nullptr || out == nullptr) {
    return false;
  }
  const size_t len = strlen(line);
  if (len >= kMaxCommandLineLen) {
    return false;  // reject rather than silently truncate/overflow
  }

  char buf[kMaxCommandLineLen];
  memcpy(buf, line, len + 1);

  constexpr uint8_t kMaxTokens = 8;
  char* tokens[kMaxTokens];
  uint8_t tokenCount = 0;

  char* saveptr = nullptr;
  char* tok = strtok_r(buf, " \t", &saveptr);
  while (tok != nullptr && tokenCount < kMaxTokens) {
    tokens[tokenCount++] = tok;
    tok = strtok_r(nullptr, " \t", &saveptr);
  }
  if (tok != nullptr) {
    return false;  // too many tokens
  }
  if (tokenCount == 0) {
    return false;
  }

  CmdType type;
  if (!commandTypeFromName(tokens[0], &type)) {
    return false;
  }

  Command cmd;
  cmd.type = type;
  cmd.seq = 0;

  switch (type) {
    case CmdType::Stop:
    case CmdType::Stand:
    case CmdType::Rest:
    case CmdType::SaveCal:
    case CmdType::Detach:
    case CmdType::EStop:
      if (tokenCount != 1) {
        return false;
      }
      break;

    case CmdType::Drive: {
      if (tokenCount != 6) {
        return false;
      }
      DrivePayload p;
      if (!parseFloat(tokens[1], &p.vx) || !parseFloat(tokens[2], &p.vy) ||
          !parseFloat(tokens[3], &p.omega) ||
          !parseFloat(tokens[4], &p.bodyHeightMm) ||
          !parseFloat(tokens[5], &p.stepHeightMm)) {
        return false;
      }
      cmd.payload.drive = p;
      break;
    }

    case CmdType::PlayClip: {
      if (tokenCount != 2) {
        return false;
      }
      PlayClipPayload p;
      if (!parseU16(tokens[1], &p.clipId)) {
        return false;
      }
      cmd.payload.playClip = p;
      break;
    }

    case CmdType::SetFace: {
      if (tokenCount != 2) {
        return false;
      }
      FaceId face;
      if (!faceIdFromName(tokens[1], &face)) {
        return false;
      }
      SetFacePayload p;
      p.faceId = uint8_t(face);
      cmd.payload.setFace = p;
      break;
    }

    case CmdType::SetJoint: {
      if (tokenCount != 3) {
        return false;
      }
      SetJointPayload p;
      if (!parseU8(tokens[1], &p.jointIndex) || p.jointIndex >= kJointCount) {
        return false;
      }
      if (!parseFloat(tokens[2], &p.deg)) {
        return false;
      }
      cmd.payload.setJoint = p;
      break;
    }

    case CmdType::SetCal: {
      if (tokenCount != 8) {
        return false;
      }
      SetCalPayload p;
      if (!parseU8(tokens[1], &p.jointIndex) || p.jointIndex >= kJointCount) {
        return false;
      }
      long invertVal = 0;
      if (!parseI16(tokens[2], &p.usMin) || !parseI16(tokens[3], &p.usMax) ||
          !parseFloat(tokens[4], &p.trimDeg) ||
          !parseLong(tokens[5], &invertVal) ||
          (invertVal != 0 && invertVal != 1) ||
          !parseFloat(tokens[6], &p.minDeg) ||
          !parseFloat(tokens[7], &p.maxDeg)) {
        return false;
      }
      p.invert = uint8_t(invertVal);
      cmd.payload.setCal = p;
      break;
    }

    case CmdType::Count:
    default:
      return false;
  }

  *out = cmd;
  return true;
}

}  // namespace core
}  // namespace sesame
