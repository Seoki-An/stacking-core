#pragma once

#include <stacking_core/planner/grasp.hpp>
#include <stacking_core/planner/manipulation.hpp>
#include <stacking_core/planner/motion.hpp>
#include <stacking_core/planner/stable_pose.hpp>

#include <memory>
#include <vector>

namespace stacking_core {

  struct regrasp_pose_problem_t {
    std::shared_ptr<BodyModel const> body_model;
    pose_t frame_from_body_pick;
    pose_t frame_from_body_place;

    // The x/y coordinates locate the stable-pose reference point. The z
    // coordinate is the handoff support-plane height.
    Vector3 handoff_position = Vector3::Zero();
  };

  struct regrasp_pose_t {
    stable_pose_t stable_pose;
    pose_t frame_from_body;
    Scalar yaw = 0.0;
  };

  struct regrasp_pose_result_t {
    solve_status_e status = solve_status_e::invalid_problem;
    std::vector<regrasp_pose_t> poses;
    planner_solver_stats_t solver;
    planner_failure_t failure;
  };

  struct regrasp_problem_t {
    phase_scene_t pick;
    phase_scene_t handoff;
    phase_scene_t place;
    motion_robot_t robot;

    // Grasp generation is intentionally separate. Pick candidates constrain
    // the pick-to-handoff leg; place candidates constrain the handoff-to-place
    // leg. Their body-relative transforms are preserved at the handoff pose.
    std::vector<grasp_candidate_t> pick_grasps;
    std::vector<grasp_candidate_t> place_grasps;
    Vector3 handoff_position = Vector3::Zero();
  };

  struct regrasp_config_t {
    stable_pose_config_t stable_pose;
    motion_planning_config_t motion;
    Vector3 approach_dir_tool = -Vector3::UnitY();
    Scalar approach_distance = 0.1;
    Scalar target_pos_tol = 2e-2;
    Scalar target_rot_tol = 0.17453292519943295;
    Scalar max_handoff_orientation_distance = 2.3561944901923448;
    int move_steps = 50;
    int grasp_steps = 50;
    int yaw_samples = 8;
    int max_candidates = 1;
  };

  [[nodiscard]] regrasp_pose_result_t generate_regrasp_poses(
    regrasp_pose_problem_t const& problem, regrasp_config_t const& config = {});

  [[nodiscard]] plan_result_t solve_regrasp(
    regrasp_problem_t const& problem, regrasp_config_t const& config = {});

}  // namespace stacking_core
