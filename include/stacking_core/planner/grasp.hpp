#pragma once

#include <stacking_core/body.hpp>
#include <stacking_core/kinematics.hpp>
#include <stacking_core/planner/types.hpp>

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

namespace stacking_core {

  enum class grasp_contact_side_e {
    left,
    right,
  };

  enum class grasp_contact_type_e {
    pad,
    tooth,
  };

  // Connects one gripper link to its immutable collision body. Entity IDs
  // must be distinct from the phase scenes and from each other.
  struct grasp_link_body_t {
    LinkId link;
    EntityId entity;
    std::shared_ptr<BodyModel const> body_model;
  };

  // Contact intent is geometry data, not a convention hidden in URDF names.
  // Geometries omitted from this list remain collision-only geometries.
  struct grasp_contact_geometry_t {
    EntityId entity;
    GeometryId geometry;
    grasp_contact_side_e side = grasp_contact_side_e::left;
    grasp_contact_type_e type = grasp_contact_type_e::pad;
    int order = 0;
    bool enforce_contact = true;
    bool contributes_force = true;
  };

  // q_hand(opening) = opening_offset + opening * opening_direction.
  // frame_from_root = frame_from_grasp * grasp_from_root.
  struct gripper_model_t {
    KinematicState state;
    LinkId root_link;
    pose_t grasp_from_root;
    Eigen::VectorXd opening_offset;
    Eigen::VectorXd opening_direction;
    Scalar opening_lower = 0.0;
    Scalar opening_upper = 1.0;
    std::vector<grasp_link_body_t> collision_bodies;
    std::vector<grasp_contact_geometry_t> contact_geometries;
  };

  struct grasp_cost_weights_t {
    Scalar antipodal_normal = 2.0;
    Scalar antipodal_position = 2.0;
    Scalar align = 10.0;
    Scalar enclosure = 10.0;
    Scalar radial_distance = 10.0;
    Scalar center_distance = 1.0;
    Scalar contact = 2.0;
    Scalar teeth_fit = 1.0;
    Scalar teeth_align = 1.0;
    Scalar flatness = 0.0;
  };

  struct grasp_score_weights_t {
    Scalar center_distance = 2.0;
    Scalar align = 2.0;
    Scalar enclosure = 2.0;
    Scalar radial_distance = 2.0;
    Scalar teeth_gap = 2.0;
    Scalar teeth_fit = 5.0;
    Scalar teeth_align = 1.0;
  };

  struct grasp_trust_region_config_t {
    int max_iters = 1000;
    Scalar subproblem_tol = 1e-2;
    Scalar radius_max = 1e-1;
    Scalar radius_init = 1.25e-2;
    Scalar radius_reduction = 0.25;
    Scalar radius_expansion = 2.0;
    Scalar gain_ratio_lower = 0.0;
    Scalar gain_ratio_upper = 0.75;
    Scalar improvement_tol = 0.0;
    Scalar step_tol = 1e-6;
  };

  struct grasp_alm_config_t {
    int max_iters = 100;
    Scalar inequality_tol = 1e-4;
    Scalar equality_tol = 1e-4;
    Scalar inequality_penalty_init = 20.0;
    Scalar equality_penalty_init = 20.0;
    Scalar inequality_penalty_increase = 5.0;
    Scalar equality_penalty_increase = 5.0;
  };

  struct grasp_force_weights_t {
    Scalar wrench = 1.0;
    Scalar complementarity = 1.0;
    Scalar cone = 1.0;
    Scalar moment = 5.0;
  };

  // Optional bilevel force-closure refinement. For every legacy disturbance
  // wrench, contact forces are minimized first; the resulting value and
  // envelope gradient are then added to the grasp objective.
  struct grasp_force_config_t {
    bool enabled = false;
    grasp_force_weights_t weight;
    Scalar friction = 1.0;
    Scalar wrench_scale = 1.0;
    Scalar damping = 1e-6;
  };

  struct grasp_generation_config_t {
    grasp_cost_weights_t cost;
    grasp_score_weights_t score;
    grasp_trust_region_config_t trust_region;
    grasp_alm_config_t alm;
    grasp_force_config_t force;
    Scalar separate_margin = 1e-2;
    Scalar plane_separate_margin = 1e-2;
    Scalar contact_margin = 1e-3;
  };

  // Every phase describes the same physical target in a potentially different
  // immutable scene. The optimized grasp is expressed in the first phase.
  struct grasp_problem_t {
    std::vector<phase_scene_t> phases;
    gripper_model_t gripper;
    grasp_t seed;
  };

  struct grasp_candidate_t {
    grasp_t grasp;
    Scalar score = 0.0;
    std::vector<grasp_contact_t> contacts;
    planner_solver_stats_t solver;
    planner_failure_t failure;
  };

  struct grasp_result_t {
    solve_status_e status = solve_status_e::invalid_problem;
    std::vector<grasp_candidate_t> candidates;
    std::optional<std::size_t> selected_index;
    planner_failure_t failure;

    [[nodiscard]] grasp_candidate_t const* selected_candidate() const noexcept;
  };

  // One manipulator state is supplied for each scene phase. All states must
  // use the same kinematic model. The solver preserves one target-relative
  // grasp while allowing a different joint configuration in every phase.
  struct joint_grasp_problem_t {
    grasp_problem_t grasp;
    std::vector<KinematicState> initial_states;
    LinkId grasp_link;
    pose_t link_from_grasp;
  };

  struct joint_grasp_candidate_t {
    grasp_candidate_t grasp;
    std::vector<Eigen::VectorXd> positions;
  };

  struct joint_grasp_result_t {
    solve_status_e status = solve_status_e::invalid_problem;
    std::vector<joint_grasp_candidate_t> candidates;
    std::optional<std::size_t> selected_index;
    planner_failure_t failure;

    [[nodiscard]] joint_grasp_candidate_t const* selected_candidate()
      const noexcept;
  };

  [[nodiscard]] grasp_result_t solve_grasp_pose(
    grasp_problem_t const& problem,
    grasp_generation_config_t const& config = {});

  [[nodiscard]] joint_grasp_result_t solve_joint_grasp(
    joint_grasp_problem_t const& problem,
    grasp_generation_config_t const& config = {});

}  // namespace stacking_core
