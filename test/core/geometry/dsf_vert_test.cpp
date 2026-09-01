#include <stacking_core/geometry/dsf_vert.hpp>

#include <cmath>
#include <numbers>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

using stacking_core::Vector3;

void require(
  bool condition,
  std::source_location location = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error(
      "DSF-Vert test requirement failed at line " +
      std::to_string(location.line()));
  }
}

bool near(Vector3 const& lhs, Vector3 const& rhs) {
  return (lhs - rhs).norm() < 1e-12;
}

stacking_core::Matrix3X octahedron_nodes(Vector3 const& offset) {
  stacking_core::Matrix3X nodes(3, 6);
  nodes.col(0) = offset + Vector3::UnitX();
  nodes.col(1) = offset - Vector3::UnitX();
  nodes.col(2) = offset + Vector3::UnitY();
  nodes.col(3) = offset - Vector3::UnitY();
  nodes.col(4) = offset + Vector3::UnitZ();
  nodes.col(5) = offset - Vector3::UnitZ();
  return nodes;
}

}  // namespace

int main() {
  using namespace stacking_core;

  DsfVertGeometry geometry {{
    .properties = {
      .id = GeometryId {10},
      .body_from_geometry =
        pose_t {Vector3 {1.0, 0.0, 0.0}, Quaternion::Identity()},
      .material = material_t {},
    },
    .nodes = octahedron_nodes(Vector3 {2.0, 0.0, 0.0}),
    .sharpness = 20,
  }};

  require(geometry.type() == geometry_type_e::dsf_vert);
  require(geometry.nodes().rowwise().mean().norm() < 1e-12);
  require(near(
    geometry.bodyFromGeometry().position, Vector3 {3.0, 0.0, 0.0}));

  pose_t const world_from_body {
    Vector3 {10.0, 0.0, 0.0}, Quaternion::Identity()};
  support_t const support_x =
    geometry.support(Vector3::UnitX(), world_from_body);
  require(std::abs(support_x.h - 14.0) < 1e-12);
  require(near(support_x.s, Vector3 {14.0, 0.0, 0.0}));

  Quaternion const quarter_turn {
    Eigen::AngleAxisd(std::numbers::pi / 2.0, Vector3::UnitZ())};
  support_t const support_y = geometry.support(
    Vector3::UnitY(), pose_t {Vector3::Zero(), quarter_turn});
  require(std::abs(support_y.h - 4.0) < 1e-12);
  require(near(support_y.s, Vector3 {0.0, 4.0, 0.0}));

  Vector3 const derivative_direction = Vector3 {1.0, 2.0, 3.0}.normalized();
  pose_t const derivative_pose {
    Vector3 {4.0, -2.0, 1.0}, quarter_turn};
  diffable_support_t const derivative_support =
    geometry.diffable_support(derivative_direction, derivative_pose);
  Matrix3 finite_difference = Matrix3::Zero();
  constexpr Scalar difference_step = 1e-6;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    Vector3 offset = Vector3::Zero();
    offset[axis] = difference_step;
    finite_difference.col(axis) =
      (geometry.support(derivative_direction + offset, derivative_pose).s -
       geometry.support(derivative_direction - offset, derivative_pose).s) /
      (2.0 * difference_step);
  }
  Scalar const derivative_error =
    (derivative_support.ds_dx - finite_difference).norm();
  if (derivative_error >= 1e-8) {
    throw std::runtime_error(
      "DSF-Vert support derivative finite-difference error: " +
      std::to_string(derivative_error));
  }
  require(derivative_support.ds_dx.isApprox(
    derivative_support.ds_dx.transpose(), 1e-12));
  require((derivative_support.ds_dx * derivative_direction)
    .norm() < 1e-10);

  Vector3 const reference_direction {1.0, 2.0, 3.0};
  diffable_support_t const reference_support =
    geometry.diffable_support(reference_direction, pose_t {});
  Scalar const reference_height_local = std::pow(
    std::pow(1.0, 20) + std::pow(2.0, 20) + std::pow(3.0, 20),
    1.0 / 20.0);
  Vector3 const reference_point_local {
    std::pow(1.0 / reference_height_local, 19),
    std::pow(2.0 / reference_height_local, 19),
    std::pow(3.0 / reference_height_local, 19),
  };
  require(std::abs(
    reference_support.h -
    (3.0 * reference_direction.x() + reference_height_local)) < 1e-12);
  require(near(
    reference_support.s,
    Vector3 {3.0, 0.0, 0.0} + reference_point_local));

  diffable_support_t const scaled_support =
    geometry.diffable_support(2.0 * reference_direction, pose_t {});
  require(near(scaled_support.s, reference_support.s));
  require(std::abs(scaled_support.h - 2.0 * reference_support.h) <
    1e-12);
  require(scaled_support.ds_dx.isApprox(
    0.5 * reference_support.ds_dx, 1e-12));

  Matrix36 pose_finite_difference = Matrix36::Zero();
  auto perturb_pose = [&](Eigen::Index axis, Scalar amount) {
    pose_t perturbed = derivative_pose;
    if (axis < 3) {
      perturbed.position[axis] += amount;
    } else {
      Vector3 rotation_vector = Vector3::Zero();
      rotation_vector[axis - 3] = amount;
      Quaternion const rotation_increment {
        Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized())};
      perturbed.orientation = normalized(
        perturbed.orientation * rotation_increment);
    }
    return perturbed;
  };
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    pose_finite_difference.col(axis) =
      (geometry.support(
         derivative_direction,
         perturb_pose(axis, difference_step)).s -
       geometry.support(
         derivative_direction,
         perturb_pose(axis, -difference_step)).s) /
      (2.0 * difference_step);
  }
  Scalar const pose_derivative_error =
    (derivative_support.ds_dq - pose_finite_difference)
      .norm();
  if (pose_derivative_error >= 1e-8) {
    throw std::runtime_error(
      "DSF-Vert support pose derivative finite-difference error: " +
      std::to_string(pose_derivative_error));
  }

  Quaternion const geometry_orientation {
    Eigen::AngleAxisd(
      0.4, Vector3 {1.0, 2.0, -1.0}.normalized())};
  DsfVertGeometry oriented_geometry {{
    .properties = {
      .id = GeometryId {12},
      .body_from_geometry = pose_t {
        Vector3 {-0.3, 0.8, 0.5}, geometry_orientation},
      .material = material_t {},
    },
    .nodes = octahedron_nodes(Vector3 {0.7, -0.2, 0.4}),
    .sharpness = 20,
  }};
  Quaternion const body_orientation {
    Eigen::AngleAxisd(
      -0.6, Vector3 {-2.0, 1.0, 3.0}.normalized())};
  pose_t const oriented_body_pose {
    Vector3 {-1.2, 2.3, 0.4}, body_orientation};
  Vector3 const oriented_direction =
    Vector3 {-0.7, 1.1, 2.4}.normalized();
  diffable_support_t const oriented_support =
    oriented_geometry.diffable_support(
      oriented_direction, oriented_body_pose);
  Matrix36 oriented_pose_finite_difference = Matrix36::Zero();
  auto perturb_oriented_pose = [&](Eigen::Index axis, Scalar amount) {
    pose_t perturbed = oriented_body_pose;
    if (axis < 3) {
      perturbed.position[axis] += amount;
    } else {
      Vector3 rotation_axis = Vector3::Zero();
      rotation_axis[axis - 3] = amount > 0.0 ? 1.0 : -1.0;
      Quaternion const rotation_increment {
        Eigen::AngleAxisd(std::abs(amount), rotation_axis)};
      perturbed.orientation = normalized(
        perturbed.orientation * rotation_increment);
    }
    return perturbed;
  };
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    oriented_pose_finite_difference.col(axis) =
      (oriented_geometry.support(
         oriented_direction,
         perturb_oriented_pose(axis, difference_step)).s -
       oriented_geometry.support(
         oriented_direction,
         perturb_oriented_pose(axis, -difference_step)).s) /
      (2.0 * difference_step);
  }
  Scalar const oriented_pose_derivative_error =
    (oriented_support.ds_dq - oriented_pose_finite_difference).norm();
  if (oriented_pose_derivative_error >= 1e-8) {
    throw std::runtime_error(
      "rotated DSF-Vert support pose derivative finite-difference error: " +
      std::to_string(oriented_pose_derivative_error));
  }

  bool rejected_empty_nodes = false;
  try {
    DsfVertGeometry const invalid_geometry {{
      .properties = {
        .id = GeometryId {11},
        .body_from_geometry = pose_t {},
        .material = material_t {},
      },
      .nodes = Matrix3X(3, 0),
    }};
    (void)invalid_geometry;
  } catch (std::invalid_argument const&) {
    rejected_empty_nodes = true;
  }
  require(rejected_empty_nodes);

  bool rejected_zero_direction = false;
  try {
    (void)geometry.support(Vector3::Zero(), pose_t {});
  } catch (std::invalid_argument const&) {
    rejected_zero_direction = true;
  }
  require(rejected_zero_direction);
}
