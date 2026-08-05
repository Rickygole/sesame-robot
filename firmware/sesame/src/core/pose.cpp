#include "pose.h"

namespace sesame {
namespace core {

JointPose JointPose::lerp(const JointPose& a, const JointPose& b, float t) {
  JointPose out;
  for (uint8_t i = 0; i < kJointCount; ++i) {
    out.deg[i] = a.deg[i] + (b.deg[i] - a.deg[i]) * t;
  }
  return out;
}

}  // namespace core
}  // namespace sesame
