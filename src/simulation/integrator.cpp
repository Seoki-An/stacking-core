#include <stacking_core/simulation/integrator.hpp>

#include <cmath>
#include <stdexcept>

namespace stacking_core {

pose_t integrate_pose(
  pose_t const& frame_from_body, motion_t const& motion, Scalar dt) {
  if (!is_valid(frame_from_body) || !is_finite(motion) ||
      !std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument(
      "pose integration requires valid state and non-negative finite dt");
  }

  Vector3 const rot_vec = motion.angular * dt;
  Scalar const angle = rot_vec.norm();
  Quaternion dq = Quaternion::Identity();
  if (angle > 0.0) {
    dq = Quaternion {Eigen::AngleAxisd(angle, rot_vec / angle)};
  }
  return pose_t {
    frame_from_body.position + motion.linear * dt,
    frame_from_body.orientation * dq,
  };
}

}  // namespace stacking_core
