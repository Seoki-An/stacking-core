#pragma once

#include <stacking_core/collision/types.hpp>
#include <stacking_core/geometry/geometry.hpp>
#include <stacking_core/scene/view.hpp>

#include <span>
#include <vector>

namespace stacking_core {

[[nodiscard]] bool supports_collision_pair(
  geometry_type_e first, geometry_type_e second) noexcept;

// Correctness baseline: expands body pairs into supported geometry pairs.
[[nodiscard]] std::vector<collision_pair_t> brute_force_middle_phase(
  SceneView const& scene,
  std::span<collision_body_pair_t const> body_pairs);

// Prunes geometry pairs using their immutable local bounds. A plane is treated
// as the boundary of its negative half-space.
[[nodiscard]] std::vector<collision_pair_t> bounding_volume_middle_phase(
  SceneView const& scene,
  std::span<collision_body_pair_t const> body_pairs,
  Scalar margin = 0.0);

}  // namespace stacking_core
