#pragma once
#include <stacking_core/planner/manipulation.hpp>

namespace stacking_core::detail {
  Vector3 scene_approach_direction(phase_scene_t const& phase,
                                   Scalar minimum_up);

  std::optional<joint_grasp_candidate_t> resolve_grasp_with_recovery(
      std::vector<phase_scene_t> const& phases, motion_robot_t const& robot,
      gripper_model_t const& gripper, grasp_candidate_t const& candidate,
      inverse_kinematics_config_t const& ik_config,
      grasp_generation_config_t const& generation, planner_failure_t& failure);

  motion_waypoint_t subgoal_waypoint(motion_robot_t const& robot,
                                     pose_t const& tool_pose,
                                     Eigen::VectorXd const& q_goal, int steps,
                                     motion_planning_config_t const& config);
} // namespace stacking_core::detail
