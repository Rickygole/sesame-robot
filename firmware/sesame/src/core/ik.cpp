#include "ik.h"

#include <cmath>

#include "mathutil.h"

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

  const float thetaRad = atan2f(px, py);
  const float rho = hypotf(px, py) - geo.coxaMm;
  const float phiRad = atan2f(-pz, rho);
  const float reach = hypotf(rho, pz);
  const float residual = reach - geo.legMm;

  IkResult result;
  result.angles.hipDeg = radToDeg(thetaRad);
  result.angles.kneeDeg = radToDeg(phiRad) - geo.kneeBendDeg;
  result.residualMm = residual;
  result.reachable = fabsf(residual) < kIkReachEpsilonMm;
  return result;
}

LegAngles anglesFromYawPitch(const LegGeometry& geo, float yawDeg, float pitchRad) {
  LegAngles angles;
  angles.hipDeg = yawDeg;
  angles.kneeDeg = radToDeg(pitchRad) - geo.kneeBendDeg;
  return angles;
}

LegAngles solveSwingDepth(const LegGeometry& geo, float yawDeg, float depthMm) {
  const float sinPhi = clampf(depthMm / geo.legMm, -1.f, 1.f, 0.f);
  const float phiRad = asinf(sinPhi);
  return anglesFromYawPitch(geo, yawDeg, phiRad);
}

}  // namespace core
}  // namespace sesame
