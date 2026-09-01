#pragma once

#include <stacking_core/planner/grasp.hpp>

#include <vector>

namespace stacking_core {

  struct grasp_simulation_problem_t {
    phase_scene_t phase;
    gripper_model_t gripper;
    grasp_t initial_grasp;
  };

  // Quasi-dynamic validation of the seven grasp coordinates
  // [translation, rotation, opening]. This is intentionally distinct from
  // rigid-body scene simulation: the target stays fixed while virtual closing
  // and approach efforts refine the gripper pose against target and obstacle
  // contacts.
  struct grasp_simulation_config_t {
    int steps = 400;
    Scalar dt = 2e-2;
    Scalar virtual_closing_effort = 2.0;
    Scalar virtual_approach_force = 5.0;
    Scalar damping = 1e-2;
    Scalar friction = 1.0;
    int pgs_iters = 80;
    Scalar error_reduction_ratio = 0.2;
    Scalar target_contact_margin = 1e-2;
    Scalar obstacle_margin = 1e-2;
    Scalar plane_obstacle_margin = 1e-2;
    Scalar active_impulse_tol = 1e-4;
    Scalar settled_velocity_tol = 1e-2;
    Scalar relaxed_velocity_tol = 1e-1;
    bool accept_relaxed = true;
    Vector3 grasp_offset_tol = Vector3 {0.5, 1.0, 0.5};
  };

  struct grasp_simulation_result_t {
    solve_status_e status = solve_status_e::invalid_problem;
    grasp_t grasp;
    std::vector<grasp_t> trajectory;
    std::vector<grasp_contact_t> contacts;
    Scalar terminal_velocity_norm = 0.0;
    bool settled = false;
    bool left_contact = false;
    bool right_contact = false;
    planner_failure_t failure;
  };

  [[nodiscard]] grasp_simulation_result_t simulate_grasp(
    grasp_simulation_problem_t const& problem,
    grasp_simulation_config_t const& config = {});

}  // namespace stacking_core
