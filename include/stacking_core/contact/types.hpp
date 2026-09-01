#pragma once

#include <stacking_core/collision/types.hpp>

namespace stacking_core {

// Orthonormal contact coordinates expressed in the scene frame. Components in
// contact coordinates are ordered (tangent_first, tangent_second, normal).
struct contact_frame_t {
  Vector3 tangent_first = Vector3::UnitX();
  Vector3 tangent_second = Vector3::UnitY();
  Vector3 normal = Vector3::UnitZ();

  [[nodiscard]] Matrix3 frameFromContact() const {
    Matrix3 result;
    result.col(0) = tangent_first;
    result.col(1) = tangent_second;
    result.col(2) = normal;
    return result;
  }
};

// Solver-independent physical interpretation of a geometric contact feature.
struct contact_t {
  contact_feature_t feature;
  contact_frame_t frame;
  Scalar friction = 0.0;
};

struct elliptical_contact_patch_t {
  Scalar semi_axis_first = 0.0;
  Scalar semi_axis_second = 0.0;
};

}  // namespace stacking_core
