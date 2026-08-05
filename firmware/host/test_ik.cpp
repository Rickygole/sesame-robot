#include <cmath>
#include <cstdio>

#include "check.h"
#include "core/geometry.h"
#include "core/ik.h"
#include "core/vec3.h"

int g_fails = 0;

using sesame::core::forwardKinematics;
using sesame::core::IkResult;
using sesame::core::kIkReachEpsilonMm;
using sesame::core::LegAngles;
using sesame::core::LegGeometry;
using sesame::core::solveFootPosition;
using sesame::core::solveSwingDepth;
using sesame::core::Vec3;

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float degToRad(float d) { return d * (kPi / 180.f); }

LegGeometry testGeo() {
  LegGeometry g;
  g.coxaMm = 25.f;
  g.legMm = 55.f;
  g.kneeBendDeg = 0.f;
  g.hipInBody = Vec3(0.f, 0.f, 0.f);
  g.lateralSign = 1.f;
  return g;
}

// Point at yaw=thetaDeg, leg elevation=phiDeg, scaled k times the leg
// length away from the coxa circle (k=1 => exactly on the torus).
Vec3 pointAtK(const LegGeometry& g, float thetaDeg, float phiDeg, float k) {
  const float theta = degToRad(thetaDeg);
  const float phi = degToRad(phiDeg);
  const float radial = g.coxaMm + k * g.legMm * std::cos(phi);
  const float height = -k * g.legMm * std::sin(phi);
  return Vec3(radial * std::sin(theta), radial * std::cos(theta), height);
}
}  // namespace

static void testRoundTripOnTorusGrid() {
  const LegGeometry g = testGeo();
  for (float thetaDeg = -45.f; thetaDeg <= 45.001f; thetaDeg += 15.f) {
    for (float phiDeg = -55.f; phiDeg <= 85.001f; phiDeg += 15.f) {
      LegAngles angles;
      angles.hipDeg = thetaDeg;
      angles.kneeDeg = phiDeg;  // kneeBendDeg == 0 here
      const Vec3 p = forwardKinematics(g, angles);

      const IkResult r = solveFootPosition(g, p);
      CHECK(r.reachable);
      CHECK_NEAR(r.residualMm, 0.0, 1e-2);
      CHECK_NEAR(r.angles.hipDeg, thetaDeg, 1e-2);
      CHECK_NEAR(r.angles.kneeDeg, phiDeg, 1e-2);

      const Vec3 back = forwardKinematics(g, r.angles);
      CHECK_NEAR(back.x, p.x, 1e-2);
      CHECK_NEAR(back.y, p.y, 1e-2);
      CHECK_NEAR(back.z, p.z, 1e-2);
    }
  }
}

static void testResidualSignAndMagnitude() {
  const LegGeometry g = testGeo();

  // Exactly on the torus.
  {
    const Vec3 p = pointAtK(g, 10.f, 20.f, 1.0f);
    const IkResult r = solveFootPosition(g, p);
    CHECK(r.reachable);
    CHECK_NEAR(r.residualMm, 0.0, 1e-2);
  }
  // Outside the torus (foot pulled farther than the leg can reach):
  // residual must be POSITIVE and match (k-1)*legMm.
  {
    const float k = 1.5f;
    const Vec3 p = pointAtK(g, -20.f, 30.f, k);
    const IkResult r = solveFootPosition(g, p);
    CHECK(!r.reachable);
    CHECK(r.residualMm > 0.f);
    CHECK_NEAR(r.residualMm, (k - 1.f) * g.legMm, 1e-2);
  }
  // Inside the torus (foot pulled closer than the leg length): residual
  // must be NEGATIVE and match (k-1)*legMm.
  {
    const float k = 0.5f;
    const Vec3 p = pointAtK(g, 5.f, -10.f, k);
    const IkResult r = solveFootPosition(g, p);
    CHECK(!r.reachable);
    CHECK(r.residualMm < 0.f);
    CHECK_NEAR(r.residualMm, (k - 1.f) * g.legMm, 1e-2);
  }
}

static void testReachabilityClassification() {
  const LegGeometry g = testGeo();
  const float kEps = kIkReachEpsilonMm;

  CHECK(solveFootPosition(g, pointAtK(g, 0.f, 0.f, 1.0f)).reachable);
  CHECK(!solveFootPosition(g, pointAtK(g, 0.f, 0.f, 1.0f + 10.f / g.legMm)).reachable);
  CHECK(!solveFootPosition(g, pointAtK(g, 0.f, 0.f, 1.0f - 10.f / g.legMm)).reachable);
  // Just inside the epsilon tolerance should still classify reachable.
  CHECK(solveFootPosition(g, pointAtK(g, 0.f, 0.f, 1.0f + 0.5f * kEps / g.legMm)).reachable);
}

static void testSolveSwingDepthExact() {
  const LegGeometry g = testGeo();
  for (float yawDeg = -40.f; yawDeg <= 40.001f; yawDeg += 20.f) {
    for (float depthMm = -50.f; depthMm <= 50.001f; depthMm += 10.f) {
      const LegAngles angles = solveSwingDepth(g, yawDeg, depthMm);
      const Vec3 p = forwardKinematics(g, angles);
      const IkResult r = solveFootPosition(g, p);
      CHECK(r.reachable);
      CHECK_NEAR(r.residualMm, 0.0, 1e-3);
      CHECK_NEAR(angles.hipDeg, yawDeg, 1e-6);
    }
  }
}

int main() {
  testRoundTripOnTorusGrid();
  testResidualSignAndMagnitude();
  testReachabilityClassification();
  testSolveSwingDepthExact();
  if (g_fails) {
    std::fprintf(stderr, "%d failure(s)\n", g_fails);
  }
  return g_fails ? 1 : 0;
}
