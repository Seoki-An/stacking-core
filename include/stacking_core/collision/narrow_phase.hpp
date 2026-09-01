#pragma once

#include <stacking_core/collision/types.hpp>
#include <stacking_core/scene/snapshot.hpp>

namespace stacking_core {

  // Computes one signed contact feature in the snapshot frame.
  [[nodiscard]] contact_feature_t compute_contact(
    SceneSnapshot const& scene, collision_pair_t pair,
    narrow_phase_config_t const& config = {});

  // Computes a DSF-DSF or plane-DSF contact feature and its pose derivatives.
  // Other geometry combinations remain available through compute_contact().
  [[nodiscard]] diffable_contact_feature_t compute_diffable_contact(
    SceneSnapshot const& scene, collision_pair_t pair,
    narrow_phase_config_t const& config = {});

}  // namespace stacking_core
