#pragma once

#include <stacking_core/types.hpp>

namespace stacking_core {

// Radially clamps (tangent_first, tangent_second, normal) while holding a
// positive normal impulse fixed. A non-positive normal impulse projects to 0.
[[nodiscard]] Vector3 project_coulomb_impulse(
  Vector3 const& impulse_contact, Scalar friction);

}  // namespace stacking_core
