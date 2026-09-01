#pragma once

#include <stacking_core/transform.hpp>

namespace stacking_core {

// Semi-implicit pose update used by diffsim. Linear velocity is expressed in
// the scene frame and angular velocity in the body frame.
[[nodiscard]] pose_t integrate_pose(
  pose_t const& frame_from_body, motion_t const& motion, Scalar dt);

}  // namespace stacking_core
