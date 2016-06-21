#ifndef VECTOR_3_F
#define VECTOR_3_F

#include "hebi_vector_3_f.h"

namespace hebi {

typedef struct Vector3f {
  float x_;
  float y_;
  float z_;

  Vector3f(float x, float y, float z) : x_(x), y_(y), z_(z) {}
  Vector3f(const HebiVector3f& src) : x_(src.x), y_(src.y), z_(src.z) {}

  float getX() const { return x_; }
  float getY() const { return y_; }
  float getZ() const { return z_; }
} Vector3f;

} // namespace hebi

#endif // VECTOR_3_F
