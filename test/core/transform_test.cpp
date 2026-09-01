#include <stacking_core/transform.hpp>

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace {

using stacking_core::Scalar;
using stacking_core::Vector3;

bool near(Vector3 const& lhs, Vector3 const& rhs) {
  return (lhs - rhs).norm() < 1e-12;
}

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("transform test requirement failed");
  }
}

}  // namespace

int main() {
  using namespace stacking_core;

  pose_t const identity;
  require(is_valid(identity));
  require(near(transform_point(identity, Vector3 {1.0, 2.0, 3.0}),
               Vector3 {1.0, 2.0, 3.0}));

  Quaternion const scaled_identity {2.0, 0.0, 0.0, 0.0};
  pose_t const normalized_pose {Vector3::Zero(), scaled_identity};
  require(is_valid(normalized_pose));
  require(std::abs(normalized_pose.orientation.norm() - 1.0) < 1e-12);

  Quaternion const quarter_turn {
    Eigen::AngleAxisd(std::numbers::pi_v<Scalar> / 2.0, Vector3::UnitZ())};
  pose_t const world_from_body {Vector3 {1.0, 2.0, 0.0}, quarter_turn};
  pose_t const body_from_geometry {Vector3 {1.0, 0.0, 0.0}, Quaternion::Identity()};
  pose_t const world_from_geometry =
    compose(world_from_body, body_from_geometry);

  require(near(world_from_geometry.position, Vector3 {1.0, 3.0, 0.0}));
  require(near(transform_point(world_from_body, Vector3 {1.0, 0.0, 0.0}),
               Vector3 {1.0, 3.0, 0.0}));
  require(near(transform_vector(world_from_body, Vector3 {1.0, 0.0, 0.0}),
               Vector3 {0.0, 1.0, 0.0}));

  pose_t const body_from_world = inverse(world_from_body);
  pose_t const round_trip = compose(body_from_world, world_from_body);
  require(near(round_trip.position, Vector3::Zero()));
  require(round_trip.orientation.isApprox(Quaternion::Identity(), 1e-12));

  motion_t finite_motion;
  require(is_finite(finite_motion));
  finite_motion.linear.x() = std::numeric_limits<Scalar>::infinity();
  require(!is_finite(finite_motion));

  bool rejected_zero_orientation = false;
  try {
    pose_t const invalid_pose {Vector3::Zero(), Quaternion {0.0, 0.0, 0.0, 0.0}};
    (void)invalid_pose;
  } catch (std::invalid_argument const&) {
    rejected_zero_orientation = true;
  }
  require(rejected_zero_orientation);
}
