#pragma once

#include <stacking_core/planner/grasp/sampling.hpp>
#include <stacking_core/planner/grasp/simulation.hpp>
#include <stacking_core/planner/motion.hpp>

#include <cstddef>
#include <optional>
#include <vector>

namespace stacking_core {

  enum class planning_stage_e {
    pick_approach,
    pick_retreat,
    transfer,
    handoff_place,
    handoff_retreat,
    handoff_approach,
    handoff_pick,
    place_approach,
    place_retreat,
  };

  struct direct_plan_problem_t {
    phase_scene_t pick;
    phase_scene_t place;
    motion_robot_t robot;
    gripper_model_t gripper;

    // Empty means that the direct planner owns sampling and joint-space
    // refinement. Supplying candidates is useful when a consumer maintains a
    // grasp cache or applies an application-specific generator.
    std::vector<joint_grasp_candidate_t> grasp_candidates;
  };

  struct inhand_plan_problem_t {
    // The current target pose is determined by robot + attachment. The place
    // phase owns the target model and the obstacle snapshot used for carry and
    // retreat planning, avoiding a second potentially conflicting scene.
    phase_scene_t place;
    motion_robot_t robot;
    attachment_t attachment;
  };

  struct direct_plan_config_t {
    grasp_sampling_config_t grasp_sampling;
    grasp_generation_config_t grasp_generation;
    inverse_kinematics_config_t inverse_kinematics;
    grasp_simulation_config_t grasp_simulation;
    motion_planning_config_t motion;
    bool simulation_refinement = true;
    Vector3 approach_dir_tool = -Vector3::UnitY();
    Scalar approach_distance = 0.1;
    Scalar target_pos_tol = 2e-2;
    Scalar target_rot_tol = 0.17453292519943295;
    int move_steps = 50;
    int grasp_steps = 50;
    int max_candidates = 1;
  };

  struct inhand_plan_config_t {
    motion_planning_config_t motion;
    Vector3 approach_dir_tool = -Vector3::UnitY();
    Scalar approach_distance = 0.1;
    Scalar target_pos_tol = 2e-2;
    Scalar target_rot_tol = 0.17453292519943295;
    int move_steps = 50;
    int grasp_steps = 50;
  };

  struct plan_segment_t {
    planning_stage_e stage = planning_stage_e::transfer;
    motion_mode_e mode = motion_mode_e::free;
    trajectory_t trajectory;
  };

  struct grasp_event_t {
    grasp_event_e event = grasp_event_e::acquire;
    std::size_t segment_index = 0;
    std::size_t sample_index = 0;
    grasp_t grasp;
  };

  struct plan_diagnostic_t {
    std::optional<planning_stage_e> stage;
    planner_failure_t failure;
  };

  struct plan_candidate_t {
    std::vector<plan_segment_t> segments;
    std::vector<grasp_event_t> grasp_events;
    Scalar score = 0.0;
    std::vector<plan_diagnostic_t> diagnostics;
  };

  struct plan_result_t {
    solve_status_e status = solve_status_e::invalid_problem;
    std::vector<plan_candidate_t> candidates;
    std::optional<std::size_t> selected_index;
    planner_failure_t failure;

    [[nodiscard]] plan_candidate_t const* selected_candidate() const noexcept;
  };

  [[nodiscard]] plan_result_t solve_direct(
    direct_plan_problem_t const& problem,
    direct_plan_config_t const& config = {});

  [[nodiscard]] plan_result_t solve_inhand(
    inhand_plan_problem_t const& problem,
    inhand_plan_config_t const& config = {});

}  // namespace stacking_core
