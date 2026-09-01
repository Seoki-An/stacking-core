#pragma once

#include <stacking_core/contact/types.hpp>
#include <stacking_core/scene/snapshot.hpp>

namespace stacking_core {

[[nodiscard]] contact_frame_t make_contact_frame(Vector3 const& normal_frame);

// Harmonic mean, matching diffsim's material-pair rule. Two frictionless
// materials combine to zero without division by zero.
[[nodiscard]] Scalar combine_friction(
  Scalar first_friction, Scalar second_friction);

// Adds a normalized contact frame and combined material friction to a
// collision feature. All values remain expressed in the snapshot frame.
[[nodiscard]] contact_t make_contact(
  SceneSnapshot const& scene, contact_feature_t feature);

}  // namespace stacking_core
