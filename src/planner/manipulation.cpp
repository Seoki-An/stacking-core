#include <stacking_core/planner/manipulation.hpp>

#include "parallel.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace stacking_core {
  namespace {

    using planner_clock_t = std::chrono::steady_clock;

    Scalar elapsed_seconds(planner_clock_t::time_point start) {
      return std::chrono::duration<Scalar>(planner_clock_t::now() - start)
        .count();
    }

    plan_result_t invalid_result(std::string code, std::string message) {
      return plan_result_t {
        .status = solve_status_e::invalid_problem,
        .candidates = {},
        .selected_index = std::nullopt,
        .failure =
          planner_failure_t {
            .code = std::move(code),
            .message = std::move(message),
            .retryable = false,
          },
        .timings = {},
      };
    }

    bool valid_phase(phase_scene_t const& phase) {
      return phase.target.valid() && phase.scene.contains(phase.target);
    }

    bool valid_direct_config(direct_plan_config_t const& config) {
      return config.approach_dir_tool.allFinite() &&
        config.approach_dir_tool.norm() > 0.0 &&
        std::isfinite(config.approach_distance) &&
        config.approach_distance >= 0.0 &&
        std::isfinite(config.target_pos_tol) && config.target_pos_tol >= 0.0 &&
        std::isfinite(config.target_rot_tol) && config.target_rot_tol >= 0.0 &&
        config.move_steps >= 2 && config.grasp_steps >= 2 &&
        config.max_candidates >= 1 && config.worker_count >= 0;
    }

    bool valid_inhand_config(inhand_plan_config_t const& config) {
      return config.approach_dir_tool.allFinite() &&
        config.approach_dir_tool.norm() > 0.0 &&
        std::isfinite(config.approach_distance) &&
        config.approach_distance >= 0.0 &&
        std::isfinite(config.target_pos_tol) && config.target_pos_tol >= 0.0 &&
        std::isfinite(config.target_rot_tol) && config.target_rot_tol >= 0.0 &&
        config.move_steps >= 2 && config.grasp_steps >= 2;
    }

    motion_robot_t robot_at(
      motion_robot_t const& robot, Eigen::VectorXd const& positions,
      Scalar opening) {
      motion_robot_t out = robot;
      out.initial_state.setPositions(positions);
      out.gripper_opening = opening;
      return out;
    }

    pose_t tool_pose_at(
      motion_robot_t const& robot, Eigen::VectorXd const& positions) {
      KinematicState state = robot.initial_state;
      state.setPositions(positions);
      KinematicSnapshot const snapshot = forward_kinematics(state);
      return compose(
        snapshot.frameFromLink(robot.tool_link), robot.link_from_tool);
    }

    pose_t link_pose_from_tool(
      motion_robot_t const& robot, pose_t const& frame_from_tool) {
      return compose(frame_from_tool, inverse(robot.link_from_tool));
    }

    motion_waypoint_t tool_waypoint(
      motion_robot_t const& robot, pose_t const& frame_from_tool, int steps) {
      return motion_waypoint_t {
        .goal =
          link_pose_goal_t {
            .link = robot.tool_link,
            .frame_from_link = link_pose_from_tool(robot, frame_from_tool),
          },
        .steps = steps,
      };
    }

    motion_waypoint_t joint_waypoint(
      Eigen::VectorXd const& positions, int steps) {
      return motion_waypoint_t {
        .goal = joint_goal_t {.positions = positions, .preserve_branch = false},
        .steps = steps,
      };
    }

    Eigen::VectorXd endpoint(motion_result_t const& motion) {
      return motion.trajectory.samples.back().robot.positions;
    }

    std::optional<pose_t> terminal_target(motion_result_t const& motion) {
      if (motion.trajectory.samples.empty()) {
        return std::nullopt;
      }
      return motion.trajectory.samples.back().frame_from_target;
    }

    attachment_t attachment_at(
      phase_scene_t const& phase, motion_robot_t const& robot,
      Eigen::VectorXd const& positions) {
      KinematicState state = robot.initial_state;
      state.setPositions(positions);
      KinematicSnapshot const snapshot = forward_kinematics(state);
      return attachment_t {
        .body = phase.target,
        .link = robot.tool_link,
        .link_from_body = compose(
          inverse(snapshot.frameFromLink(robot.tool_link)),
          phase.scene.body(phase.target).frameFromBody()),
      };
    }

    phase_scene_t phase_with_target_pose(
      phase_scene_t const& phase, pose_t const& frame_from_target) {
      std::vector<EntityId> ids(
        phase.scene.entityIds().begin(), phase.scene.entityIds().end());
      std::vector<BodyInstance> bodies;
      bodies.reserve(ids.size());
      for (EntityId id : ids) {
        BodyInstance const& source = phase.scene.body(id);
        bodies.emplace_back(body_instance_config_t {
          .id = source.id(),
          .model = source.modelPtr(),
          .frame_from_body =
            id == phase.target ? frame_from_target : source.frameFromBody(),
          .motion = source.motion(),
          .mobility = source.mobility(),
        });
      }
      auto snapshot = std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
        .frame = phase.scene.frame(),
        .bodies = std::move(bodies),
      });
      return phase_scene_t {
        .scene = SceneView {std::move(snapshot), std::move(ids)},
        .target = phase.target,
      };
    }

    void set_static_target(trajectory_t& trajectory, pose_t const& pose) {
      for (trajectory_sample_t& sample : trajectory.samples) {
        sample.frame_from_target = pose;
      }
    }

    bool pose_close(
      pose_t const& lhs, pose_t const& rhs, Scalar pos_tol, Scalar rot_tol) {
      return (lhs.position - rhs.position).norm() <= pos_tol &&
        lhs.orientation.angularDistance(rhs.orientation) <= rot_tol;
    }

    planner_failure_t stage_failure(
      char const* prefix, planning_stage_e stage,
      motion_result_t const& motion) {
      auto stage_name = [](planning_stage_e value) {
        switch (value) {
        case planning_stage_e::pick_approach:
          return "pick_approach";
        case planning_stage_e::pick_retreat:
          return "pick_retreat";
        case planning_stage_e::transfer:
          return "transfer";
        case planning_stage_e::place_approach:
          return "place_approach";
        case planning_stage_e::place_retreat:
          return "place_retreat";
        default:
          return "motion";
        }
      };
      return planner_failure_t {
        .code = std::string {prefix} + "_" + stage_name(stage) + "_motion",
        .message = motion.failure.message.empty()
          ? "manipulation motion did not satisfy its boundary"
          : motion.failure.message,
        .retryable = motion.failure.retryable,
      };
    }

    planner_failure_t target_boundary_failure(char const* prefix) {
      return planner_failure_t {
        .code = std::string {prefix} + "_place_target_boundary",
        .message = "attached motion did not place the target within tolerance",
        .retryable = true,
      };
    }

    bool valid_joint_candidate(
      joint_grasp_candidate_t const& candidate, Eigen::Index dof) {
      return is_valid(candidate.grasp.grasp.frame_from_grasp) &&
        std::isfinite(candidate.grasp.grasp.opening) &&
        std::isfinite(candidate.grasp.score) &&
        candidate.grasp.failure.code.empty() &&
        candidate.positions.size() == 2 &&
        std::all_of(
               candidate.positions.begin(), candidate.positions.end(),
               [&](Eigen::VectorXd const& q) {
                 return q.size() == dof && q.allFinite();
               });
    }

    pose_t phase_grasp_pose(
      phase_scene_t const& ref, phase_scene_t const& phase,
      pose_t const& frame_from_grasp) {
      pose_t const phase_from_ref = compose(
        phase.scene.body(phase.target).frameFromBody(),
        inverse(ref.scene.body(ref.target).frameFromBody()));
      return compose(phase_from_ref, frame_from_grasp);
    }

    std::optional<joint_grasp_candidate_t> resolve_grasp_ik(
      direct_plan_problem_t const& problem, grasp_candidate_t candidate,
      direct_plan_config_t const& config, planner_failure_t& failure) {
      std::vector<Eigen::VectorXd> positions;
      positions.reserve(2);
      for (phase_scene_t const* phase : {&problem.pick, &problem.place}) {
        pose_t const frame_from_grasp = phase_grasp_pose(
          problem.pick, *phase, candidate.grasp.frame_from_grasp);
        pose_t const frame_from_link = compose(
          frame_from_grasp, inverse(problem.robot.link_from_tool));
        KinematicState state = problem.robot.initial_state;
        inverse_kinematics_config_t ik_config = config.inverse_kinematics;
        if (problem.robot.ik_initializer) {
          std::optional<Eigen::VectorXd> const initialized =
            problem.robot.ik_initializer(
              state, problem.robot.tool_link, frame_from_link);
          if (
            !initialized.has_value() ||
            initialized->size() != state.positions().size() ||
            !initialized->allFinite()) {
            failure = planner_failure_t {
              .code = "direct_grasp_ik_initializer_failed",
              .message = "direct grasp IK initializer returned an invalid seed",
              .retryable = true,
            };
            return std::nullopt;
          }
          state.setPositions(*initialized);
          ik_config.initialization =
            inverse_kinematics_initialization_e::provided;
        }
        inverse_kinematics_result_t const ik = solve_inverse_kinematics(
          inverse_kinematics_problem_t {
            .initial_state = std::move(state),
            .link = problem.robot.tool_link,
            .frame_from_link = frame_from_link,
            .position_only = false,
          },
          ik_config);
        if (ik.status != solve_status_e::success) {
          failure = ik.failure;
          return std::nullopt;
        }
        positions.push_back(ik.positions);
      }
      return joint_grasp_candidate_t {
        .grasp = std::move(candidate),
        .positions = std::move(positions),
      };
    }

    std::optional<joint_grasp_candidate_t> simulation_refine(
      direct_plan_problem_t const& problem,
      joint_grasp_candidate_t const& candidate,
      direct_plan_config_t const& config, planner_failure_t& failure) {
      if (!config.simulation_refinement) {
        return candidate;
      }
      grasp_simulation_result_t const simulated = simulate_grasp(
        grasp_simulation_problem_t {
          .phase = problem.pick,
          .gripper = problem.gripper,
          .initial_grasp = candidate.grasp.grasp,
        },
        config.grasp_simulation);
      if (simulated.status != solve_status_e::success) {
        failure = simulated.failure;
        return std::nullopt;
      }
      grasp_candidate_t refined = candidate.grasp;
      refined.grasp = simulated.grasp;
      return resolve_grasp_ik(
        problem, std::move(refined), config, failure);
    }

    struct direct_candidate_result_t {
      solve_status_e status = solve_status_e::infeasible;
      std::optional<plan_candidate_t> candidate;
      planner_failure_t failure;
    };

    direct_candidate_result_t solve_direct_candidate(
      direct_plan_problem_t const& problem,
      joint_grasp_candidate_t const& refined,
      direct_plan_config_t const& config) {
      BodyInstance const& pick_target =
        problem.pick.scene.body(problem.pick.target);
      BodyInstance const& place_target =
        problem.place.scene.body(problem.place.target);
      Vector3 const approach_axis = config.approach_dir_tool.normalized();
      Eigen::VectorXd const q_home = problem.robot.initial_state.positions();
      Eigen::VectorXd const& q_pick = refined.positions[0];
      Eigen::VectorXd const& q_place = refined.positions[1];
      pose_t const grasp_pick = tool_pose_at(problem.robot, q_pick);
      pose_t const grasp_place = tool_pose_at(problem.robot, q_place);
      direct_candidate_result_t out;

      for (Scalar scale : std::array<Scalar, 4> {1.0, 0.75, 0.5, 0.25}) {
        pose_t pre_pick = grasp_pick;
        pre_pick.position += scale * config.approach_distance *
          transform_vector(grasp_pick, approach_axis);
        pose_t pre_place = grasp_place;
        pre_place.position += scale * config.approach_distance *
          transform_vector(grasp_place, approach_axis);

        motion_result_t pick_approach = solve_free_motion(
          free_motion_problem_t {
            .scene = problem.pick.scene,
            .robot =
              robot_at(problem.robot, q_home, problem.robot.gripper_opening),
            .waypoints =
              {tool_waypoint(problem.robot, pre_pick, config.move_steps),
               joint_waypoint(q_pick, config.grasp_steps)},
          },
          config.motion);
        if (pick_approach.status == solve_status_e::invalid_problem) {
          out.status = solve_status_e::invalid_problem;
          out.failure = pick_approach.failure;
          return out;
        }
        if (pick_approach.status != solve_status_e::success) {
          out.failure = stage_failure(
            "direct", planning_stage_e::pick_approach, pick_approach);
          continue;
        }
        set_static_target(
          pick_approach.trajectory, pick_target.frameFromBody());
        Eigen::VectorXd const q_at_pick = endpoint(pick_approach);
        attachment_t const pick_attachment =
          attachment_at(problem.pick, problem.robot, q_at_pick);

        motion_result_t pick_retreat = solve_grasped_motion(
          grasped_motion_problem_t {
            .scene = problem.pick.scene,
            .robot =
              robot_at(problem.robot, q_at_pick, refined.grasp.grasp.opening),
            .attachment = pick_attachment,
            .waypoints = {tool_waypoint(
              problem.robot, pre_pick, config.grasp_steps)},
          },
          config.motion);
        if (pick_retreat.status == solve_status_e::invalid_problem) {
          out.status = solve_status_e::invalid_problem;
          out.failure = pick_retreat.failure;
          return out;
        }
        if (pick_retreat.status != solve_status_e::success) {
          out.failure = stage_failure(
            "direct", planning_stage_e::pick_retreat, pick_retreat);
          continue;
        }

        attachment_t place_attachment = pick_attachment;
        place_attachment.body = problem.place.target;
        motion_result_t transfer = solve_grasped_motion(
          grasped_motion_problem_t {
            .scene = problem.place.scene,
            .robot = robot_at(
              problem.robot, endpoint(pick_retreat),
              refined.grasp.grasp.opening),
            .attachment = place_attachment,
            .waypoints = {tool_waypoint(
              problem.robot, pre_place, config.move_steps)},
          },
          config.motion);
        if (transfer.status == solve_status_e::invalid_problem) {
          out.status = solve_status_e::invalid_problem;
          out.failure = transfer.failure;
          return out;
        }
        if (transfer.status != solve_status_e::success) {
          out.failure =
            stage_failure("direct", planning_stage_e::transfer, transfer);
          continue;
        }

        motion_result_t place_approach = solve_grasped_motion(
          grasped_motion_problem_t {
            .scene = problem.place.scene,
            .robot = robot_at(
              problem.robot, endpoint(transfer), refined.grasp.grasp.opening),
            .attachment = place_attachment,
            .waypoints = {joint_waypoint(q_place, config.grasp_steps)},
          },
          config.motion);
        if (place_approach.status == solve_status_e::invalid_problem) {
          out.status = solve_status_e::invalid_problem;
          out.failure = place_approach.failure;
          return out;
        }
        std::optional<pose_t> const released_target =
          terminal_target(place_approach);
        if (place_approach.status != solve_status_e::success) {
          out.failure = stage_failure(
            "direct", planning_stage_e::place_approach, place_approach);
          continue;
        }
        if (
          !released_target.has_value() ||
          !pose_close(
            *released_target, place_target.frameFromBody(),
            config.target_pos_tol, config.target_rot_tol)) {
          out.failure = target_boundary_failure("direct");
          continue;
        }
        Eigen::VectorXd const q_at_place = endpoint(place_approach);
        phase_scene_t const released_phase =
          phase_with_target_pose(problem.place, *released_target);
        motion_result_t place_retreat = solve_free_motion(
          free_motion_problem_t {
            .scene = released_phase.scene,
            .robot = robot_at(
              problem.robot, q_at_place, problem.robot.gripper_opening),
            .waypoints =
              {tool_waypoint(problem.robot, pre_place, config.grasp_steps),
               joint_waypoint(q_home, config.move_steps)},
          },
          config.motion);
        if (place_retreat.status == solve_status_e::invalid_problem) {
          out.status = solve_status_e::invalid_problem;
          out.failure = place_retreat.failure;
          return out;
        }
        if (place_retreat.status != solve_status_e::success) {
          out.failure = stage_failure(
            "direct", planning_stage_e::place_retreat, place_retreat);
          continue;
        }
        set_static_target(place_retreat.trajectory, *released_target);

        plan_candidate_t candidate;
        candidate.score = refined.grasp.score;
        candidate.segments = {
          plan_segment_t {
            .stage = planning_stage_e::pick_approach,
            .mode = motion_mode_e::free,
            .trajectory = std::move(pick_approach.trajectory),
          },
          plan_segment_t {
            .stage = planning_stage_e::pick_retreat,
            .mode = motion_mode_e::attached,
            .trajectory = std::move(pick_retreat.trajectory),
          },
          plan_segment_t {
            .stage = planning_stage_e::transfer,
            .mode = motion_mode_e::attached,
            .trajectory = std::move(transfer.trajectory),
          },
          plan_segment_t {
            .stage = planning_stage_e::place_approach,
            .mode = motion_mode_e::attached,
            .trajectory = std::move(place_approach.trajectory),
          },
          plan_segment_t {
            .stage = planning_stage_e::place_retreat,
            .mode = motion_mode_e::free,
            .trajectory = std::move(place_retreat.trajectory),
          }};
        candidate.grasp_events = {
          grasp_event_t {
            .event = grasp_event_e::acquire,
            .segment_index = 0,
            .sample_index = candidate.segments[0].trajectory.samples.size() - 1,
            .grasp =
              grasp_t {
                .frame_from_grasp = tool_pose_at(problem.robot, q_at_pick),
                .opening = refined.grasp.grasp.opening,
              },
          },
          grasp_event_t {
            .event = grasp_event_e::release,
            .segment_index = 3,
            .sample_index = candidate.segments[3].trajectory.samples.size() - 1,
            .grasp =
              grasp_t {
                .frame_from_grasp = tool_pose_at(problem.robot, q_at_place),
                .opening = refined.grasp.grasp.opening,
              },
          }};
        out.status = solve_status_e::success;
        out.candidate = std::move(candidate);
        out.failure = {};
        return out;
      }
      return out;
    }

    std::vector<joint_grasp_candidate_t> direct_grasps(
      direct_plan_problem_t const& problem, direct_plan_config_t const& config,
      planner_failure_t& failure, solve_status_e& status) {
      if (!problem.grasp_candidates.empty()) {
        status = solve_status_e::success;
        return problem.grasp_candidates;
      }
      grasp_sampling_config_t sampling_config = config.grasp_sampling;
      sampling_config.worker_count = config.worker_count;
      grasp_result_t sampled = sample_grasps(
        grasp_sampling_problem_t {
          .phases = {problem.pick, problem.place},
          .gripper = problem.gripper,
        },
        sampling_config, config.grasp_generation);
      failure = sampled.failure;
      status = sampled.status;
      if (status == solve_status_e::invalid_problem) {
        return {};
      }

      std::vector<joint_grasp_candidate_t> reachable;
      reachable.reserve(sampled.candidates.size());
      for (grasp_candidate_t& candidate : sampled.candidates) {
        if (!candidate.failure.code.empty()) {
          continue;
        }
        planner_failure_t ik_failure;
        std::optional<joint_grasp_candidate_t> resolved = resolve_grasp_ik(
          problem, std::move(candidate), config, ik_failure);
        if (resolved.has_value()) {
          reachable.push_back(std::move(*resolved));
          continue;
        }
        if (!ik_failure.retryable) {
          failure = std::move(ik_failure);
          status = solve_status_e::invalid_problem;
          return {};
        }
        failure = std::move(ik_failure);
      }
      if (reachable.empty()) {
        status = solve_status_e::infeasible;
        failure = planner_failure_t {
          .code = "sampled_grasps_unreachable",
          .message = "no pose-space grasp was reachable at pick and place",
          .retryable = true,
        };
      } else {
        status = solve_status_e::success;
        failure = {};
      }
      return reachable;
    }

  }  // namespace

  plan_result_t solve_direct(
    direct_plan_problem_t const& problem, direct_plan_config_t const& config) {
    if (!valid_direct_config(config)) {
      return invalid_result(
        "invalid_direct_config", "direct config is invalid");
    }
    if (!valid_phase(problem.pick) || !valid_phase(problem.place)) {
      return invalid_result(
        "invalid_direct_phase", "pick and place must contain their targets");
    }
    if (
      problem.pick.scene.frame() != problem.place.scene.frame() ||
      problem.pick.scene.frame() != problem.robot.initial_state.frame()) {
      return invalid_result(
        "direct_frame_mismatch", "pick, place, and robot must share one frame");
    }
    BodyInstance const& pick_target =
      problem.pick.scene.body(problem.pick.target);
    BodyInstance const& place_target =
      problem.place.scene.body(problem.place.target);
    if (pick_target.model().id() != place_target.model().id()) {
      return invalid_result(
        "direct_target_model_mismatch",
        "pick and place targets must share a body model");
    }

    planner_clock_t::time_point const total_start = planner_clock_t::now();
    planner_clock_t::time_point const grasp_start = planner_clock_t::now();
    planner_failure_t grasp_failure;
    solve_status_e grasp_status = solve_status_e::infeasible;
    std::vector<joint_grasp_candidate_t> grasps =
      direct_grasps(problem, config, grasp_failure, grasp_status);
    if (grasp_status == solve_status_e::invalid_problem) {
      plan_result_t result =
        invalid_result(grasp_failure.code, grasp_failure.message);
      result.timings.grasp_generation_seconds = elapsed_seconds(grasp_start);
      result.timings.total_seconds = elapsed_seconds(total_start);
      result.timings.grasp_candidates = grasps.size();
      return result;
    }

    plan_result_t out {
      .status = solve_status_e::infeasible,
      .candidates = {},
      .selected_index = std::nullopt,
      .failure =
        planner_failure_t {
          .code = "direct_grasp_generation",
          .message = grasp_failure.message.empty()
            ? "no common pick/place grasp was feasible"
            : grasp_failure.message,
          .retryable = true,
        },
      .timings = {},
    };
    out.timings.grasp_generation_seconds = elapsed_seconds(grasp_start);
    out.timings.grasp_candidates = grasps.size();
    Eigen::Index const dof = problem.robot.initial_state.positions().size();
    grasps.erase(
      std::remove_if(
        grasps.begin(), grasps.end(),
        [&](joint_grasp_candidate_t const& candidate) {
          return !valid_joint_candidate(candidate, dof);
        }),
      grasps.end());
    std::stable_sort(
      grasps.begin(), grasps.end(),
      [](
        joint_grasp_candidate_t const& lhs,
        joint_grasp_candidate_t const& rhs) {
        return lhs.grasp.score > rhs.grasp.score;
      });

    planner_clock_t::time_point const refinement_start = planner_clock_t::now();
    std::vector<std::optional<joint_grasp_candidate_t>> refined_results(
      grasps.size());
    std::vector<planner_failure_t> refinement_failures(grasps.size());
    detail::planner_parallel_for(
      grasps.size(), config.worker_count, [&](std::size_t i) {
        refined_results[i] =
          simulation_refine(problem, grasps[i], config, refinement_failures[i]);
      });
    out.timings.simulation_refinement_seconds =
      elapsed_seconds(refinement_start);

    std::vector<joint_grasp_candidate_t> refined;
    refined.reserve(grasps.size());
    for (std::size_t i = 0; i < refined_results.size(); ++i) {
      if (refined_results[i].has_value()) {
        refined.push_back(std::move(*refined_results[i]));
        continue;
      }
      planner_failure_t const& failure = refinement_failures[i];
      if (!failure.code.empty() && !failure.retryable) {
        out.status = solve_status_e::invalid_problem;
        out.failure = failure;
        out.timings.total_seconds = elapsed_seconds(total_start);
        return out;
      }
      if (!failure.code.empty()) {
        out.failure = failure;
      }
    }
    out.timings.refined_candidates = refined.size();

    planner_clock_t::time_point const motion_start = planner_clock_t::now();
    std::vector<std::optional<direct_candidate_result_t>> motion_results(
      refined.size());
    std::atomic<std::size_t> accepted_count {0};
    std::atomic<std::size_t> attempted_count {0};
    auto enough_candidates = [&]() {
      return accepted_count.load(std::memory_order_relaxed) >=
        static_cast<std::size_t>(config.max_candidates);
    };
    detail::planner_parallel_for(
      refined.size(), config.worker_count, enough_candidates,
      [&](std::size_t i) {
        attempted_count.fetch_add(1, std::memory_order_relaxed);
        direct_candidate_result_t result =
          solve_direct_candidate(problem, refined[i], config);
        bool const accepted = result.status == solve_status_e::success;
        motion_results[i] = std::move(result);
        if (accepted) {
          accepted_count.fetch_add(1, std::memory_order_relaxed);
        }
      });
    out.timings.trajectory_optimization_seconds = elapsed_seconds(motion_start);
    out.timings.motion_candidates =
      attempted_count.load(std::memory_order_relaxed);

    for (std::optional<direct_candidate_result_t>& value : motion_results) {
      if (
        out.candidates.size() >=
        static_cast<std::size_t>(config.max_candidates)) {
        break;
      }
      if (!value.has_value()) {
        continue;
      }
      if (value->status == solve_status_e::invalid_problem) {
        out.status = solve_status_e::invalid_problem;
        out.candidates.clear();
        out.selected_index = std::nullopt;
        out.failure = std::move(value->failure);
        out.timings.total_seconds = elapsed_seconds(total_start);
        return out;
      }
      if (value->candidate.has_value()) {
        out.candidates.push_back(std::move(*value->candidate));
      } else if (!value->failure.code.empty()) {
        out.failure = std::move(value->failure);
      }
    }
    if (!out.candidates.empty()) {
      std::stable_sort(
        out.candidates.begin(), out.candidates.end(),
        [](plan_candidate_t const& lhs, plan_candidate_t const& rhs) {
          return lhs.score > rhs.score;
        });
      out.status = solve_status_e::success;
      out.selected_index = 0;
      out.failure = {};
    }
    out.timings.total_seconds = elapsed_seconds(total_start);
    return out;
  }

  plan_result_t solve_inhand(
    inhand_plan_problem_t const& problem, inhand_plan_config_t const& config) {
    if (!valid_inhand_config(config)) {
      return invalid_result(
        "invalid_inhand_config", "in-hand config is invalid");
    }
    if (
      !valid_phase(problem.place) ||
      problem.attachment.body != problem.place.target ||
      problem.attachment.link != problem.robot.tool_link ||
      !is_valid(problem.attachment.link_from_body)) {
      return invalid_result(
        "invalid_inhand_problem",
        "in-hand planning requires a valid tool attachment and place target");
    }
    if (problem.place.scene.frame() != problem.robot.initial_state.frame()) {
      return invalid_result(
        "inhand_frame_mismatch", "place scene and robot must share one frame");
    }
    BodyInstance const& target = problem.place.scene.body(problem.place.target);

    pose_t const frame_from_link = compose(
      target.frameFromBody(), inverse(problem.attachment.link_from_body));
    pose_t const frame_from_tool =
      compose(frame_from_link, problem.robot.link_from_tool);
    Vector3 const approach_axis = config.approach_dir_tool.normalized();
    Eigen::VectorXd const q_home = problem.robot.initial_state.positions();
    attachment_t place_attachment = problem.attachment;
    place_attachment.body = problem.place.target;
    plan_result_t out {
      .status = solve_status_e::infeasible,
      .candidates = {},
      .selected_index = std::nullopt,
      .failure =
        planner_failure_t {
          .code = "inhand_motion_infeasible",
          .message = "no approach distance produced a feasible in-hand place",
          .retryable = true,
        },
      .timings = {},
    };

    for (Scalar scale : std::array<Scalar, 4> {1.0, 0.75, 0.5, 0.25}) {
      pose_t pre_place = frame_from_tool;
      pre_place.position += scale * config.approach_distance *
        transform_vector(frame_from_tool, approach_axis);
      motion_result_t transfer = solve_grasped_motion(
        grasped_motion_problem_t {
          .scene = problem.place.scene,
          .robot = problem.robot,
          .attachment = place_attachment,
          .waypoints = {tool_waypoint(
            problem.robot, pre_place, config.move_steps)},
        },
        config.motion);
      if (transfer.status == solve_status_e::invalid_problem) {
        return invalid_result(transfer.failure.code, transfer.failure.message);
      }
      if (transfer.status != solve_status_e::success) {
        out.failure =
          stage_failure("inhand", planning_stage_e::transfer, transfer);
        continue;
      }
      motion_result_t place_approach = solve_grasped_motion(
        grasped_motion_problem_t {
          .scene = problem.place.scene,
          .robot = robot_at(
            problem.robot, endpoint(transfer), problem.robot.gripper_opening),
          .attachment = place_attachment,
          .waypoints = {motion_waypoint_t {
            .goal =
              link_pose_goal_t {
                .link = problem.robot.tool_link,
                .frame_from_link = frame_from_link,
              },
            .steps = config.grasp_steps,
          }},
        },
        config.motion);
      if (place_approach.status == solve_status_e::invalid_problem) {
        return invalid_result(
          place_approach.failure.code, place_approach.failure.message);
      }
      std::optional<pose_t> const released_target =
        terminal_target(place_approach);
      if (place_approach.status != solve_status_e::success) {
        out.failure = stage_failure(
          "inhand", planning_stage_e::place_approach, place_approach);
        continue;
      }
      if (
        !released_target.has_value() ||
        !pose_close(
          *released_target, target.frameFromBody(), config.target_pos_tol,
          config.target_rot_tol)) {
        out.failure = target_boundary_failure("inhand");
        continue;
      }
      Eigen::VectorXd const q_place = endpoint(place_approach);
      phase_scene_t const released_phase =
        phase_with_target_pose(problem.place, *released_target);
      motion_result_t place_retreat = solve_free_motion(
        free_motion_problem_t {
          .scene = released_phase.scene,
          .robot =
            robot_at(problem.robot, q_place, problem.robot.gripper_opening),
          .waypoints =
            {tool_waypoint(problem.robot, pre_place, config.grasp_steps),
             joint_waypoint(q_home, config.move_steps)},
        },
        config.motion);
      if (place_retreat.status == solve_status_e::invalid_problem) {
        return invalid_result(
          place_retreat.failure.code, place_retreat.failure.message);
      }
      if (place_retreat.status != solve_status_e::success) {
        out.failure = stage_failure(
          "inhand", planning_stage_e::place_retreat, place_retreat);
        continue;
      }
      set_static_target(place_retreat.trajectory, *released_target);

      plan_candidate_t candidate;
      candidate.segments = {
        plan_segment_t {
          .stage = planning_stage_e::transfer,
          .mode = motion_mode_e::attached,
          .trajectory = std::move(transfer.trajectory),
        },
        plan_segment_t {
          .stage = planning_stage_e::place_approach,
          .mode = motion_mode_e::attached,
          .trajectory = std::move(place_approach.trajectory),
        },
        plan_segment_t {
          .stage = planning_stage_e::place_retreat,
          .mode = motion_mode_e::free,
          .trajectory = std::move(place_retreat.trajectory),
        }};
      candidate.grasp_events = {grasp_event_t {
        .event = grasp_event_e::release,
        .segment_index = 1,
        .sample_index = candidate.segments[1].trajectory.samples.size() - 1,
        .grasp =
          grasp_t {
            .frame_from_grasp = tool_pose_at(problem.robot, q_place),
            .opening = problem.robot.gripper_opening,
          },
      }};
      out.status = solve_status_e::success;
      out.candidates.push_back(std::move(candidate));
      out.selected_index = 0;
      out.failure = {};
      return out;
    }
    return out;
  }

}  // namespace stacking_core
