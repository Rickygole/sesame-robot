#include "geometry.h"

#include <cmath>

namespace sesame {
namespace core {

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float degToRad(float deg) { return deg * (kPi / 180.f); }
}  // namespace

Vec3 radialUnit(float thetaRad) {
  return Vec3(std::sin(thetaRad), std::cos(thetaRad), 0.f);
}

Vec3 forwardKinematics(const LegGeometry& geo, LegAngles angles) {
  const float theta = degToRad(angles.hipDeg);
  const float phi = degToRad(angles.kneeDeg + geo.kneeBendDeg);
  const Vec3 u = radialUnit(theta);
  const float radial = geo.coxaMm + geo.legMm * std::cos(phi);
  const float height = -geo.legMm * std::sin(phi);
  return Vec3(radial * u.x, radial * u.y, height);
}

Vec3 footBodyToHipFrame(const LegGeometry& geo, Vec3 footInBody) {
  const Vec3 rel = footInBody - geo.hipInBody;
  return Vec3(rel.x, geo.lateralSign * rel.y, rel.z);
}

Vec3 footHipFrameToBody(const LegGeometry& geo, Vec3 footInHipFrame) {
  const Vec3 rel(footInHipFrame.x, geo.lateralSign * footInHipFrame.y,
                  footInHipFrame.z);
  return rel + geo.hipInBody;
}

}  // namespace core
}  // namespace sesame
