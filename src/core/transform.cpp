#include <stacking_core/transform.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace stacking_core {
namespace {

constexpr Scalar quaternion_norm_tolerance = 1e-12;

bool is_finite(Quaternion const& orientation) noexcept {
  return orientation.coeffs().allFinite();
}

}  // namespace

pose_t::pose_t(Vector3 position_, Quaternion orientation_)
    : position(std::move(position_)),
      orientation(normalized(std::move(orientation_))) {
  if (!position.allFinite()) {
    throw std::invalid_argument("pose position must be finite");
  }
}

Quaternion normalized(Quaternion orientation) {
  Scalar const norm = orientation.norm();
  if (!is_finite(orientation) || !std::isfinite(norm) || norm <= 0.0) {
    throw std::invalid_argument("pose orientation must be finite and non-zero");
  }
  orientation.coeffs() /= norm;
  return orientation;
}

bool is_valid(pose_t const& pose) noexcept {
  if (!pose.position.allFinite() || !is_finite(pose.orientation)) {
    return false;
  }
  return std::abs(pose.orientation.squaredNorm() - 1.0) <=
    quaternion_norm_tolerance;
}

bool is_finite(motion_t const& motion) noexcept {
  return motion.linear.allFinite() && motion.angular.allFinite();
}

pose_t compose(pose_t const& a_from_b, pose_t const& b_from_c) {
  Quaternion const a_rotation_from_b = normalized(a_from_b.orientation);
  Quaternion const b_rotation_from_c = normalized(b_from_c.orientation);
  return pose_t {
    a_from_b.position + a_rotation_from_b * b_from_c.position,
    a_rotation_from_b * b_rotation_from_c,
  };
}

pose_t inverse(pose_t const& a_from_b) {
  Quaternion const b_rotation_from_a = normalized(a_from_b.orientation).conjugate();
  return pose_t {
    -(b_rotation_from_a * a_from_b.position),
    b_rotation_from_a,
  };
}

Vector3 transform_point(pose_t const& a_from_b, Vector3 const& point_b) {
  return a_from_b.position + normalized(a_from_b.orientation) * point_b;
}

Vector3 transform_vector(pose_t const& a_from_b, Vector3 const& vector_b) {
  return normalized(a_from_b.orientation) * vector_b;
}

Matrix3 skew(Vector3 const& vector) {
  Matrix3 result;
  result << 0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  return result;
}

}  // namespace stacking_core
