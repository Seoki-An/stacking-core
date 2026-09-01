#pragma once

#include <stacking_core/types.hpp>

#include <compare>

namespace stacking_core {

struct geometry_instance_id_t {
  EntityId entity;
  GeometryId geometry;

  [[nodiscard]] bool valid() const noexcept {
    return entity.valid() && geometry.valid();
  }

  auto operator<=>(geometry_instance_id_t const&) const = default;
};

struct collision_body_pair_t {
  EntityId first;
  EntityId second;
};

struct collision_pair_t {
  geometry_instance_id_t first;
  geometry_instance_id_t second;
};

struct contact_feature_t {
  collision_pair_t pair;

  // Positive when separated, zero when touching, negative when penetrating.
  Scalar gap = 0.0;
  Vector3 point_first = Vector3::Zero();
  Vector3 point_second = Vector3::Zero();

  // Unit vector in the scene frame, directed from first toward second.
  Vector3 normal = Vector3::UnitZ();

  [[nodiscard]] bool penetrating() const noexcept {
    return gap < 0.0;
  }
};

// Derivatives are with respect to right pose perturbations [q_first, q_second],
// where each q is [translation_in_scene, rotation_in_body].
struct diffable_contact_feature_t: contact_feature_t {
  Eigen::Matrix<Scalar, 1, 12> d_gap =
    Eigen::Matrix<Scalar, 1, 12>::Zero();
  Eigen::Matrix<Scalar, 3, 12> d_point_first =
    Eigen::Matrix<Scalar, 3, 12>::Zero();
  Eigen::Matrix<Scalar, 3, 12> d_point_second =
    Eigen::Matrix<Scalar, 3, 12>::Zero();
  Eigen::Matrix<Scalar, 3, 12> d_normal =
    Eigen::Matrix<Scalar, 3, 12>::Zero();
};

struct narrow_phase_config_t {
  int max_dir_iters = 50;
  Scalar dir_tol = 1e-6;
  Scalar tr_radius_init = 10.0;
  Scalar tr_subproblem_tol = 0.1;
  Scalar gain_ratio_lower_thresh = 0.05;
  Scalar gain_ratio_upper_thresh = 0.9;
  Scalar tr_radius_reduction_rate = 0.25;
  Scalar tr_radius_expansion_rate = 3.0;
};

}  // namespace stacking_core
