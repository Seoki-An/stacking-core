#include "recovery.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stacking_core/collision.hpp>
#include <stacking_core/geometry/plane.hpp>
#include <stdexcept>

namespace stacking_core::detail {
  Vector3 scene_approach_direction(phase_scene_t const& phase,
                                   Scalar minimum_up) {
    if (!std::isfinite(minimum_up) || minimum_up < 0.0 || minimum_up > 1.0)
      throw std::invalid_argument(
          "minimum approach up component must be in [0,1]");
    Vector3 plane_sum = Vector3::Zero();
    BodyInstance const& target = phase.scene.body(phase.target);
    std::vector<collision_body_pair_t> bodies;
    for (EntityId id : phase.scene.entityIds()) {
      if (id == phase.target)
        continue;
      auto const& body = phase.scene.body(id);
      bool plane = false;
      for (std::size_t i = 0; i < body.model().geometryCount(); ++i) {
        auto const& geometry = body.model().geometry(i);
        if (geometry.type() == geometry_type_e::plane) {
          plane = true;
          plane_sum += static_cast<PlaneGeometry const&>(geometry).normal(
              body.frameFromBody());
        }
      }
      Scalar distance =
          (body.frameFromBody().position - target.frameFromBody().position)
              .norm();
      if (plane || (distance >= 1e-6 && distance <= 1.5))
        bodies.push_back({phase.target, id});
    }
    Vector3 up = plane_sum.norm() > 1e-9 ? Vector3(plane_sum.normalized())
                                         : Vector3::UnitZ();
    Vector3 direction = plane_sum.norm() > 1e-9 ? plane_sum : up;
    for (auto const& pair : brute_force_middle_phase(phase.scene, bodies)) {
      auto const feature =
          compute_diffable_contact(phase.scene.snapshot(), pair);
      if (std::isfinite(feature.gap) && feature.gap < 0.10) {
        // Middle phase may order the bodies: use the target's derivative block.
        if (pair.first.entity == phase.target)
          direction += feature.d_gap.leftCols<3>().transpose();
        else
          direction += feature.d_gap.middleCols<3>(6).transpose();
      }
    }
    if (direction.norm() < 1e-9)
      return up;
    direction.normalize();
    Scalar vertical = direction.dot(up);
    if (vertical < minimum_up) {
      Vector3 horizontal = direction - vertical * up;
      if (horizontal.norm() < 1e-9)
        return up;
      direction = minimum_up * up + std::sqrt(1.0 - minimum_up * minimum_up) *
                                        horizontal.normalized();
    }
    return direction;
  }

  std::optional<joint_grasp_candidate_t> resolve_grasp_with_recovery(
      std::vector<phase_scene_t> const& phases, motion_robot_t const& robot,
      gripper_model_t const& gripper, grasp_candidate_t const& candidate,
      inverse_kinematics_config_t const& ik_config,
      grasp_generation_config_t const& generation, planner_failure_t& failure) {
    std::vector<KinematicState> states;
    bool reachable = true;
    bool first_phase_reachable = false;
    auto const body_grasp = compose(
        inverse(
            phases.front().scene.body(phases.front().target).frameFromBody()),
        candidate.grasp.frame_from_grasp);
    for (auto const& phase : phases) {
      auto state = robot.initial_state;
      auto local_config = ik_config;
      auto pose = compose(
          compose(phase.scene.body(phase.target).frameFromBody(), body_grasp),
          inverse(robot.link_from_tool));
      if (robot.ik_initializer) {
        auto seed = robot.ik_initializer(state, robot.tool_link, pose);
        if (seed && seed->size() == state.positions().size() &&
            seed->allFinite()) {
          state.setPositions(*seed);
          local_config.initialization =
              inverse_kinematics_initialization_e::provided;
        }
      }
      auto ik = solve_inverse_kinematics({state, robot.tool_link, pose, false},
                                         local_config);
      if (states.empty())
        first_phase_reachable = ik.status == solve_status_e::success;
      reachable = reachable && ik.status == solve_status_e::success;
      if (ik.positions.size() == state.positions().size() &&
          ik.positions.allFinite())
        state.setPositions(ik.positions);
      states.push_back(std::move(state));
    }
    if (reachable) {
      joint_grasp_candidate_t result{.grasp = candidate};
      for (auto const& state : states)
        result.positions.push_back(state.positions());
      failure = {};
      return result;
    }
    // Legacy first recovers the fixed pick/place grasp, then constrains the
    // same body-relative grasp at both endpoints. The first solve supplies a
    // seed only; the final solve still checks every scene and joint limit.
    grasp_t seed = candidate.grasp;
    if (!first_phase_reachable && phases.size() > 1) {
      auto first = solve_joint_grasp({{{phases.front()}, gripper, seed},
                                      {states.front()},
                                      robot.tool_link,
                                      robot.link_from_tool},
                                     generation);
      if (first.status == solve_status_e::invalid_problem) {
        failure = first.failure;
        return std::nullopt;
      }
      if (first.status == solve_status_e::success &&
          first.selected_candidate()) {
        auto const& recovered = *first.selected_candidate();
        states.front().setPositions(recovered.positions.front());
        seed = recovered.grasp.grasp;
        first_phase_reachable = true;
      }
    }
    auto result = solve_joint_grasp({{phases, gripper, seed},
                                     states,
                                     robot.tool_link,
                                     robot.link_from_tool},
                                    generation);
    if (result.status != solve_status_e::success ||
        !result.selected_candidate()) {
      failure = result.failure;
      if (failure.retryable) {
        failure.code = first_phase_reachable ? "grasp_ik_other_phase"
                                             : "grasp_ik_first_phase";
      }
      return std::nullopt;
    }
    failure = {};
    return *result.selected_candidate();
  }

  motion_waypoint_t subgoal_waypoint(motion_robot_t const& robot,
                                     pose_t const& tool_pose,
                                     Eigen::VectorXd const& q_goal, int steps,
                                     motion_planning_config_t const& config) {
    auto state = robot.initial_state;
    Eigen::VectorXd seed = (q_goal + state.positions()) / 2.0;
    auto const& model = state.model();
    for (std::size_t j = 0; j < model.jointCount(); ++j) {
      auto const& joint = model.joint(j);
      auto index = model.degreeOfFreedomIndex(joint.id);
      if (index && !joint.mimic && joint.limit &&
          joint.limit->upper - joint.limit->lower >= 2.0 * std::numbers::pi)
        seed(*index) = q_goal(*index);
    }
    state.setPositions(seed);
    auto pose = compose(tool_pose, inverse(robot.link_from_tool));
    auto ik = solve_inverse_kinematics(
        {state, robot.tool_link, pose, false},
        {.max_iters = config.ik_max_iters,
         .tol = config.ik_tol,
         .initialization = inverse_kinematics_initialization_e::provided});
    if (ik.status == solve_status_e::success) {
      for (std::size_t j = 0; j < model.jointCount(); ++j) {
        auto const& joint = model.joint(j);
        auto index = model.degreeOfFreedomIndex(joint.id);
        if (!index || joint.mimic || !joint.limit ||
            joint.limit->upper - joint.limit->lower < 2.0 * std::numbers::pi)
          continue;
        Scalar proposed =
            ik.positions(*index) +
            2.0 * std::numbers::pi *
                std::round((q_goal(*index) - ik.positions(*index)) /
                           (2.0 * std::numbers::pi));
        if (proposed >= joint.limit->lower && proposed <= joint.limit->upper)
          ik.positions(*index) = proposed;
      }
      return {joint_goal_t{.positions = ik.positions, .preserve_branch = true},
              steps};
    }
    // Preserve strict IK feasibility; the normal waypoint resolver can retry.
    return {link_pose_goal_t{robot.tool_link, pose}, steps};
  }
} // namespace stacking_core::detail
