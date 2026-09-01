#pragma once

#include <stacking_core/contact/types.hpp>

namespace stacking_core {

struct limit_surface_projection_config_t {
  int max_iters = 20;
  Scalar tol = 1e-5;
  Scalar friction_ratio_thresh = 1e-2;
};

// Torsional coefficient for a uniform-pressure elliptical patch. The returned
// value has units of length times friction coefficient.
[[nodiscard]] Scalar torsional_friction_coefficient(
  Scalar friction, elliptical_contact_patch_t const& patch);

// Projects (tangent_first, tangent_second, normal, torsion) onto the 4D
// ellipsoidal friction limit surface while holding a positive normal impulse
// fixed. A non-positive normal impulse projects to 0.
[[nodiscard]] Vector4 project_limit_surface_impulse(
  Vector4 const& impulse_contact,
  Scalar friction,
  Scalar torsional_friction,
  limit_surface_projection_config_t const& config = {});

}  // namespace stacking_core
