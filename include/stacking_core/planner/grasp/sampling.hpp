#pragma once

#include <stacking_core/planner/grasp.hpp>
#include <stacking_core/planner/inverse_kinematics.hpp>

#include <cstddef>
#include <vector>

namespace stacking_core {

  enum class grasp_sampling_strategy_e {
    antipodal_surface,
    parallel_jaw,
  };

  struct grasp_sampling_problem_t {
    std::vector<phase_scene_t> phases;
    gripper_model_t gripper;
  };

  struct grasp_sampling_config_t {
    grasp_sampling_strategy_e strategy =
      grasp_sampling_strategy_e::antipodal_surface;
    int max_seeds = 30;
    int dir_samples = 40;
    int spin_samples = 3;
    Scalar spin_step = 1.0471975511965976;
    bool include_flipped = true;

    // Positive distances move the gripper opposite its approach direction.
    Scalar retreat_distance = 0.0;

    // Zero disables an explicit target-width cap. Parallel-jaw sampling still
    // respects the maximum aperture measured from the gripper model.
    Scalar max_target_width = 0.0;
    Scalar aperture_margin = 1e-2;

    // Zero disables the support-point clearance filter.
    Scalar scene_clearance_margin = 0.0;

    // Optional preferred gripper-Z direction for a nominal parallel-jaw seed.
    // Zero derives the frame from the first and last target orientations.
    Vector3 preferred_parallel_axis = Vector3::Zero();

    // Zero selects the available hardware concurrency. One preserves serial
    // execution. Positive values cap the number of seed-solving workers.
    int worker_count = 0;

    // Zero evaluates every generated seed. A positive value stops dispatching
    // new seeds after this many distinct feasible candidates are found.
    int max_candidates = 0;
  };

  struct grasp_seed_t {
    grasp_t grasp;
    Vector3 contact_positive = Vector3::Zero();
    Vector3 contact_negative = Vector3::Zero();
  };

  struct grasp_seed_result_t {
    solve_status_e status = solve_status_e::invalid_problem;
    std::vector<grasp_seed_t> seeds;
    std::size_t rejected_width = 0;
    std::size_t rejected_clearance = 0;
    planner_failure_t failure;
  };

  struct joint_grasp_sampling_problem_t {
    grasp_sampling_problem_t grasp;
    std::vector<KinematicState> initial_states;
    LinkId grasp_link;
    pose_t link_from_grasp;
    inverse_kinematics_initializer_t ik_initializer;
  };

  [[nodiscard]] grasp_seed_result_t generate_grasp_seeds(
    grasp_sampling_problem_t const& problem,
    grasp_sampling_config_t const& config = {});

  // Generates seeds, refines each one with solve_grasp_pose(), ranks feasible
  // candidates by score, and removes converged duplicates.
  [[nodiscard]] grasp_result_t sample_grasps(
    grasp_sampling_problem_t const& problem,
    grasp_sampling_config_t const& sampling_config = {},
    grasp_generation_config_t const& generation_config = {});

  // Runs IK from every sampled pose before joint-space grasp refinement.
  [[nodiscard]] joint_grasp_result_t sample_joint_grasps(
    joint_grasp_sampling_problem_t const& problem,
    grasp_sampling_config_t const& sampling_config = {},
    grasp_generation_config_t const& generation_config = {},
    inverse_kinematics_config_t const& ik_config = {});

}  // namespace stacking_core
