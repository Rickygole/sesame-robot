// vec3.h -- minimal 3D float vector. Pure math, no dependencies.
#pragma once

namespace sesame {
namespace core {

struct Vec3 {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;

  Vec3() : x(0.f), y(0.f), z(0.f) {}
  Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

  Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
  Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
  Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
  Vec3& operator+=(const Vec3& o) {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }

}  // namespace core
}  // namespace sesame
