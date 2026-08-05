#include "ik.h"

#include <cmath>

namespace sesame {
namespace core {

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float radToDeg(float rad) { return rad * (180.f / kPi); }
}  // namespace

IkResult solveFootPosition(const LegGeometry& geo, Vec3 footInHipFrame) {
  const float px = footInHipFrame.x;
  const float py = footInHipFrame.y;
  const float pz = footInHipFrame.z;

  const float thetaRad = std::atan2(px, py);
  const float rho = std::hypot(px, py) - geo.coxaMm;
  const float phiRad = std::atan2(-pz, rho);
  const float reach = std::hypot(rho, pz);
  const float residual = reach - geo.legMm;

  IkResult result;
  result.angles.hipDeg = radToDeg(thetaRad);
  result.angles.kneeDeg = radToDeg(phiRad) - geo.kneeBendDeg;
  result.residualMm = residual;
  result.reachable = std::fabs(residual) < kIkReachEpsilonMm;
  return result;
}

LegAngles solveSwingDepth(const LegGeometry& geo, float yawDeg, float depthMm) {
  float sinPhi = depthMm / geo.legMm;
  if (sinPhi > 1.f) sinPhi = 1.f;
  if (sinPhi < -1.f) sinPhi = -1.f;
  const float phiRad = std::asin(sinPhi);

  LegAngles angles;
  angles.hipDeg = yawDeg;
  angles.kneeDeg = radToDeg(phiRad) - geo.kneeBendDeg;
  return angles;
}

}  // namespace core
}  // namespace sesame
