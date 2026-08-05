// pose.h -- a full-body joint pose in LOGICAL joint order (leg*2+joint).
#pragma once

#include "types.h"

namespace sesame {
namespace core {

// ANATOMICAL degrees, one per logical joint (see types.h::logicalIndex).
// hip: + = swing forward. knee: + = foot down.
struct JointPose {
  float deg[kJointCount] = {};

  static JointPose lerp(const JointPose& a, const JointPose& b, float t);
};

}  // namespace core
}  // namespace sesame
