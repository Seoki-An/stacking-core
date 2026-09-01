#pragma once

#include <stacking_core/collision/types.hpp>
#include <stacking_core/scene/view.hpp>

#include <vector>

namespace stacking_core {

// Correctness baseline: enumerates every distinct body pair in deterministic
// SceneView order. No mobility filtering is applied.
[[nodiscard]] std::vector<collision_body_pair_t> brute_force_broad_phase(
  SceneView const& scene);

// Prunes body pairs using immutable body-local bounds transformed into the
// scene frame. Empty bodies never produce candidates.
[[nodiscard]] std::vector<collision_body_pair_t> bounding_volume_broad_phase(
  SceneView const& scene,
  Scalar margin = 0.0);

}  // namespace stacking_core
