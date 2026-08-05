// types.h -- core logical/wire joint addressing.
//
// See src/core/README.md for the purity contract this whole directory
// must obey. This header defines the ONE place where the logical joint
// order (leg*2+joint) is translated to the physical wire/GPIO order.
#pragma once

#include <stdint.h>

namespace sesame {
namespace core {

enum class Leg : uint8_t { FR = 0, RR = 1, FL = 2, RL = 3, Count = 4 };
enum class Joint : uint8_t { Hip = 0, Knee = 1 };

constexpr uint8_t kLegCount = 4;
constexpr uint8_t kJointsPerLeg = 2;
constexpr uint8_t kJointCount = 8;

// Logical joint index: leg*2 + joint. This is the order used by every
// API above the servo-channel layer (JointPose, calibration arrays, etc).
constexpr uint8_t logicalIndex(Leg l, Joint j) {
  return uint8_t(uint8_t(l) * 2 + uint8_t(j));
}

struct JointAddr {
  Leg leg;
  Joint joint;
};

// Wire channel -> logical joint. This table encodes the physical wiring
// (see repo README pin map) and is the ONLY place the 4/5 channel
// swap (rear-right knee before front-right knee) appears anywhere in
// this codebase.
constexpr JointAddr kChannelMap[kJointCount] = {
    {Leg::FR, Joint::Hip},   // ch0 GPIO15 R1
    {Leg::RR, Joint::Hip},   // ch1 GPIO2  R2
    {Leg::FL, Joint::Hip},   // ch2 GPIO23 L1
    {Leg::RL, Joint::Hip},   // ch3 GPIO19 L2
    {Leg::RR, Joint::Knee},  // ch4 GPIO4  R4
    {Leg::FR, Joint::Knee},  // ch5 GPIO16 R3
    {Leg::FL, Joint::Knee},  // ch6 GPIO17 L3
    {Leg::RL, Joint::Knee},  // ch7 GPIO18 L4
};

namespace detail {
// Recursive constexpr search (C++11 forbids multi-statement constexpr
// bodies, so this has to be an expression, not a loop).
constexpr uint8_t findChannel(Leg l, Joint j, uint8_t ch) {
  return (ch >= kJointCount)
             ? uint8_t(0xFF)
             : ((kChannelMap[ch].leg == l && kChannelMap[ch].joint == j)
                    ? ch
                    : findChannel(l, j, uint8_t(ch + 1)));
}
}  // namespace detail

// Compile-time inverse of kChannelMap: logical (leg,joint) -> wire channel.
constexpr uint8_t channelFor(Leg l, Joint j) {
  return detail::findChannel(l, j, 0);
}

}  // namespace core
}  // namespace sesame
