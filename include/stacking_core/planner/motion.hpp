#pragma once

#include <stacking_core/kinematics.hpp>
#include <stacking_core/planner/inverse_kinematics.hpp>
#include <stacking_core/planner/types.hpp>

#include <memory>
#include <optional>
#include <variant>
#include <vector>

namespace stacking_core {

  struct joint_goal_t {
    Eigen::VectorXd positions;
    bool preserve_branch = false;
  };

  struct link_pose_goal_t {
    LinkId link;
    pose_t frame_from_link;
  };

  using motion_goal_t = std::variant<joint_goal_t, link_pose_goal_t>;

  struct motion_waypoint_t {
    motion_goal_t goal;
    int steps = 0;
  };

  // Connects one kinematic link to its immutable collision model. Entity IDs
  // must be distinct from the obstacle scene and from each other.
  struct motion_link_body_t {
    LinkId link;
    EntityId entity;
    std::shared_ptr<BodyModel const> body_model;
  };

  struct motion_robot_t {
    KinematicState initial_state;
    LinkId tool_link;
    // Fixed transform from the terminal kinematic link to the task/tool frame.
    pose_t link_from_tool;
    Scalar gripper_opening = 0.0;
    // Optional robot-specific seed. The generic planner owns only LM
    // refinement; hard-coded closed-form models plug in here without becoming
    // dependencies of stacking-core::planner.
    inverse_kinematics_initializer_t ik_initializer;
    std::vector<motion_link_body_t> collision_bodies;

    // Only explicitly listed pairs are checked for self-collision. This avoids
    // treating adjacent links, whose collision meshes normally overlap at a
    // joint, as obstacles.
    std::vector<collision_body_pair_t> self_collision_pairs;
  };

  struct free_motion_problem_t {
    SceneView scene;
    motion_robot_t robot;
    std::vector<motion_waypoint_t> waypoints;
  };

  struct grasped_motion_problem_t {
    SceneView scene;
    motion_robot_t robot;
    attachment_t attachment;
    std::vector<motion_waypoint_t> waypoints;
  };

  struct motion_planning_config_t {
    Scalar smooth_weight = 1000.0;
    Scalar boundary_weight = 100.0;
    Scalar collision_weight = 1000.0;
    Scalar joint_limit_weight = 1000.0;
    Scalar smooth_boundary_alpha = 2.0;
    Scalar swing_smoothness_scale = 1.0;
    Scalar collision_margin = 2e-2;
    Scalar plane_feasibility_margin = -1e-3;
    Scalar joint_limit_margin = 1e-3;
    Scalar grasped_boundary_pos_scale = 10.0;
    Scalar grasped_boundary_rot_scale = 1.0;
    Scalar target_collision_tol = 5e-3;
    // Legacy Huber clearance penalty; <= 0 keeps the quadratic penalty.
    Scalar collision_penetration_clamp = 0.0;
    // Shared free/grasped PHR outer loop. These are optimization controls,
    // not additional allowances in the final collision feasibility check.
    bool collision_alm_enabled = false;
    int collision_alm_max_iters = 5;
    Scalar collision_alm_beta_init = 0.0;  // <= 0 uses collision_weight.
    Scalar collision_alm_beta_increase = 2.0;
    Scalar collision_alm_tol = 1e-3;
    Scalar step_size = 1e-2;
    int max_iters = 500;
    Scalar tol = 1e-6;
    int ik_max_iters = 5000;
    Scalar ik_tol = 1e-4;
  };

  struct motion_result_t {
    solve_status_e status = solve_status_e::invalid_problem;
    trajectory_t trajectory;
    planner_solver_stats_t solver;
    planner_failure_t failure;
  };

  [[nodiscard]] motion_result_t solve_free_motion(
    free_motion_problem_t const& problem,
    motion_planning_config_t const& config = {});

  [[nodiscard]] motion_result_t solve_grasped_motion(
    grasped_motion_problem_t const& problem,
    motion_planning_config_t const& config = {});

}  // namespace stacking_core
