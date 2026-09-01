#pragma once

#include <stacking_core/types.hpp>

namespace stacking_core {

// A pose named a_from_b maps coordinates expressed in frame b into frame a.
struct pose_t {
  Vector3 position = Vector3::Zero();
  Quaternion orientation = Quaternion::Identity();

  pose_t() = default;
  pose_t(Vector3 position, Quaternion orientation);
};

struct motion_t {
  Vector3 linear = Vector3::Zero();
  Vector3 angular = Vector3::Zero();
};

[[nodiscard]] Quaternion normalized(Quaternion orientation);
[[nodiscard]] bool is_valid(pose_t const& pose) noexcept;
[[nodiscard]] bool is_finite(motion_t const& motion) noexcept;

// compose(a_from_b, b_from_c) returns a_from_c.
[[nodiscard]] pose_t compose(pose_t const& a_from_b, pose_t const& b_from_c);
[[nodiscard]] pose_t inverse(pose_t const& a_from_b);

[[nodiscard]] Vector3 transform_point(
  pose_t const& a_from_b, Vector3 const& point_b);
[[nodiscard]] Vector3 transform_vector(
  pose_t const& a_from_b, Vector3 const& vector_b);

[[nodiscard]] Matrix3 skew(Vector3 const& vector);

}  // namespace stacking_core
