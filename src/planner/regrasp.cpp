#include <stacking_core/geometry/dsf_vert.hpp>
#include <stacking_core/planner/regrasp.hpp>

#include "parallel.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
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

    struct regrasp_leg_t {
      bool feasible = false;
      solve_status_e status = solve_status_e::infeasible;
      std::vector<plan_segment_t> segments;
      std::vector<grasp_event_t> events;
      Scalar score = 0.0;
      planner_failure_t failure;
    };

    regrasp_pose_result_t invalid_pose_result(
      std::string code, std::string message) {
      return regrasp_pose_result_t {
        .status = solve_status_e::invalid_problem,
        .poses = {},
        .solver = {},
        .failure =
          {
            .code = std::move(code),
            .message = std::move(message),
            .retryable = false,
          },
      };
    }

    plan_result_t invalid_plan_result(std::string code, std::string message) {
      return plan_result_t {
        .status = solve_status_e::invalid_problem,
        .candidates = {},
        .selected_index = std::nullopt,
        .failure =
          {
            .code = std::move(code),
            .message = std::move(message),
            .retryable = false,
          },
        .timings = {},
      };
    }

    bool valid_config(regrasp_config_t const& config) {
      return config.approach_dir_tool.allFinite() &&
        config.approach_dir_tool.norm() > 0.0 &&
        std::isfinite(config.approach_distance) &&
        config.approach_distance >= 0.0 &&
        std::isfinite(config.target_pos_tol) && config.target_pos_tol >= 0.0 &&
        std::isfinite(config.target_rot_tol) && config.target_rot_tol >= 0.0 &&
        std::isfinite(config.max_handoff_orientation_distance) &&
        config.max_handoff_orientation_distance >= 0.0 &&
        config.max_handoff_orientation_distance <= std::numbers::pi &&
        config.move_steps >= 2 && config.grasp_steps >= 2 &&
        config.yaw_samples >= 1 && config.max_candidates >= 1 &&
        config.worker_count >= 0;
    }

    Matrix3 basis_from_z(Vector3 z) {
      z.normalize();
      Vector3 x;
      if (z.x() != 0.0 || z.y() != 0.0) {
        x = Vector3 {-z.y(), z.x(), 0.0};
      } else {
        x = Vector3 {0.0, -z.z(), z.y()};
      }
      x.normalize();
      Matrix3 R;
      R.col(0) = x;
      R.col(1) = z.cross(x);
      R.col(2) = z;
      return R;
    }

    Scalar support_height_about(
      BodyModel const& body, Vector3 const& dir, Vector3 const& ref_body) {
      Scalar height = -std::numeric_limits<Scalar>::infinity();
      for (std::size_t i = 0; i < body.geometryCount(); ++i) {
        Geometry const& geometry = body.geometry(i);
        if (geometry.type() != geometry_type_e::dsf_vert) {
          continue;
        }
        auto const& dsf = static_cast<DsfVertGeometry const&>(geometry);
        support_t const support =
          dsf.support(dir, pose_t {-ref_body, Quaternion::Identity()});
        height = std::max(height, support.h);
      }
      return height;
    }

    pose_t resting_pose(
      BodyModel const& body, stable_pose_t const& stable,
      Vector3 const& ref_body, Vector3 const& handoff_position, Scalar yaw) {
      Matrix3 const R_0 = basis_from_z(-stable.resting_dir_body).transpose();
      Matrix3 const R =
        Eigen::AngleAxis<Scalar> {yaw, Vector3::UnitZ()}.toRotationMatrix() *
        R_0;
      Scalar const height =
        support_height_about(body, stable.resting_dir_body, ref_body);
      Vector3 ref_frame = handoff_position;
      ref_frame.z() += height;
      return pose_t {ref_frame - R * ref_body, Quaternion {R}};
    }

    bool pose_close(
      pose_t const& first, pose_t const& second,
      regrasp_config_t const& config) {
      return (first.position - second.position).norm() <=
        config.target_pos_tol &&
        first.orientation.angularDistance(second.orientation) <=
        config.target_rot_tol;
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
          .id = id,
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

    motion_robot_t robot_at(
      motion_robot_t const& robot, Eigen::VectorXd const& positions,
      Scalar opening) {
      motion_robot_t result = robot;
      result.initial_state.setPositions(positions);
      result.gripper_opening = opening;
      return result;
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
        .goal =
          joint_goal_t {
            .positions = positions,
            .preserve_branch = false,
          },
        .steps = steps,
      };
    }

    pose_t tool_pose_at(
      motion_robot_t const& robot, Eigen::VectorXd const& positions) {
      KinematicState state = robot.initial_state;
      state.setPositions(positions);
      KinematicSnapshot const snapshot = forward_kinematics(state);
      return compose(
        snapshot.frameFromLink(robot.tool_link), robot.link_from_tool);
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

    Eigen::VectorXd endpoint(motion_result_t const& motion) {
      return motion.trajectory.samples.back().robot.positions;
    }

    void set_static_target(trajectory_t& trajectory, pose_t const& pose) {
      for (trajectory_sample_t& sample : trajectory.samples) {
        sample.frame_from_target = pose;
      }
    }

    planner_failure_t stage_failure(
      planning_stage_e stage, motion_result_t const& motion) {
      auto stage_code = [](planning_stage_e value) {
        switch (value) {
        case planning_stage_e::pick_approach:
          return "regrasp_pick_approach_motion";
        case planning_stage_e::pick_retreat:
          return "regrasp_pick_retreat_motion";
        case planning_stage_e::transfer:
          return "regrasp_transfer_motion";
        case planning_stage_e::handoff_place:
          return "regrasp_to_handoff_motion";
        case planning_stage_e::handoff_retreat:
          return "regrasp_handoff_retreat_motion";
        case planning_stage_e::handoff_approach:
          return "regrasp_handoff_approach_motion";
        case planning_stage_e::handoff_pick:
          return "regrasp_from_handoff_motion";
        case planning_stage_e::place_approach:
          return "regrasp_place_approach_motion";
        case planning_stage_e::place_retreat:
          return "regrasp_place_retreat_motion";
        }
        return "regrasp_motion";
      };
      return planner_failure_t {
        .code = stage_code(stage),
        .message = motion.failure.message.empty()
          ? "regrasp motion did not satisfy its target boundary"
          : motion.failure.message,
        .retryable = motion.failure.retryable,
      };
    }

    std::optional<pose_t> terminal_target(motion_result_t const& motion) {
      if (motion.trajectory.samples.empty()) {
        return std::nullopt;
      }
      return motion.trajectory.samples.back().frame_from_target;
    }

    regrasp_leg_t solve_pick_leg(
      regrasp_problem_t const& problem, phase_scene_t const& handoff,
      grasp_candidate_t const& candidate, regrasp_config_t const& config) {
      regrasp_leg_t last;
      BodyInstance const& pick_target =
        problem.pick.scene.body(problem.pick.target);
      BodyInstance const& handoff_target = handoff.scene.body(handoff.target);
      pose_t const body_from_grasp = compose(
        inverse(pick_target.frameFromBody()), candidate.grasp.frame_from_grasp);
      pose_t const grasp_pick = candidate.grasp.frame_from_grasp;
      pose_t const grasp_handoff =
        compose(handoff_target.frameFromBody(), body_from_grasp);
      Vector3 const approach_axis = config.approach_dir_tool.normalized();

      for (Scalar scale : std::array<Scalar, 4> {1.0, 0.75, 0.5, 0.25}) {
        pose_t approach_pick = grasp_pick;
        approach_pick.position += scale * config.approach_distance *
          transform_vector(grasp_pick, approach_axis);
        pose_t approach_handoff = grasp_handoff;
        approach_handoff.position += scale * config.approach_distance *
          transform_vector(grasp_handoff, approach_axis);

        regrasp_leg_t leg;
        leg.score = candidate.score;
        Eigen::VectorXd const q_home = problem.robot.initial_state.positions();
        motion_result_t approach = solve_free_motion(
          free_motion_problem_t {
            .scene = problem.pick.scene,
            .robot =
              robot_at(problem.robot, q_home, problem.robot.gripper_opening),
            .waypoints =
              {
                tool_waypoint(problem.robot, approach_pick, config.move_steps),
                tool_waypoint(problem.robot, grasp_pick, config.grasp_steps),
              },
          },
          config.motion);
        if (approach.status != solve_status_e::success) {
          leg.status = approach.status;
          leg.failure =
            stage_failure(planning_stage_e::pick_approach, approach);
          last = std::move(leg);
          continue;
        }
        set_static_target(approach.trajectory, pick_target.frameFromBody());
        Eigen::VectorXd const q_pick = endpoint(approach);
        attachment_t const pick_attachment =
          attachment_at(problem.pick, problem.robot, q_pick);

        motion_result_t retreat = solve_grasped_motion(
          grasped_motion_problem_t {
            .scene = problem.pick.scene,
            .robot = robot_at(problem.robot, q_pick, candidate.grasp.opening),
            .attachment = pick_attachment,
            .waypoints = {tool_waypoint(
              problem.robot, approach_pick, config.grasp_steps)},
          },
          config.motion);
        if (retreat.status != solve_status_e::success) {
          leg.status = retreat.status;
          leg.failure = stage_failure(planning_stage_e::pick_retreat, retreat);
          last = std::move(leg);
          continue;
        }

        attachment_t handoff_attachment = pick_attachment;
        handoff_attachment.body = handoff.target;
        motion_result_t handoff_place = solve_grasped_motion(
          grasped_motion_problem_t {
            .scene = handoff.scene,
            .robot = robot_at(
              problem.robot, endpoint(retreat), candidate.grasp.opening),
            .attachment = handoff_attachment,
            .waypoints =
              {
                tool_waypoint(
                  problem.robot, approach_handoff, config.move_steps),
                tool_waypoint(problem.robot, grasp_handoff, config.grasp_steps),
              },
          },
          config.motion);
        std::optional<pose_t> const released_target =
          terminal_target(handoff_place);
        if (
          handoff_place.status != solve_status_e::success ||
          !released_target.has_value() ||
          !pose_close(
            *released_target, handoff_target.frameFromBody(), config)) {
          leg.status = handoff_place.status == solve_status_e::success
            ? solve_status_e::infeasible
            : handoff_place.status;
          leg.failure =
            stage_failure(planning_stage_e::handoff_place, handoff_place);
          last = std::move(leg);
          continue;
        }
        Eigen::VectorXd const q_release = endpoint(handoff_place);

        motion_result_t handoff_retreat = solve_free_motion(
          free_motion_problem_t {
            .scene = handoff.scene,
            .robot =
              robot_at(problem.robot, q_release, problem.robot.gripper_opening),
            .waypoints =
              {
                tool_waypoint(
                  problem.robot, approach_handoff, config.grasp_steps),
                joint_waypoint(q_home, config.move_steps),
              },
          },
          config.motion);
        if (handoff_retreat.status != solve_status_e::success) {
          leg.status = handoff_retreat.status;
          leg.failure =
            stage_failure(planning_stage_e::handoff_retreat, handoff_retreat);
          last = std::move(leg);
          continue;
        }
        set_static_target(
          handoff_retreat.trajectory, handoff_target.frameFromBody());

        leg.segments = {
          plan_segment_t {
            .stage = planning_stage_e::pick_approach,
            .mode = motion_mode_e::free,
            .trajectory = std::move(approach.trajectory),
          },
          plan_segment_t {
            .stage = planning_stage_e::pick_retreat,
            .mode = motion_mode_e::attached,
            .trajectory = std::move(retreat.trajectory),
          },
          plan_segment_t {
            .stage = planning_stage_e::handoff_place,
            .mode = motion_mode_e::attached,
            .trajectory = std::move(handoff_place.trajectory),
          },
          plan_segment_t {
            .stage = planning_stage_e::handoff_retreat,
            .mode = motion_mode_e::free,
            .trajectory = std::move(handoff_retreat.trajectory),
          },
        };
        leg.events = {
          grasp_event_t {
            .event = grasp_event_e::acquire,
            .segment_index = 0,
            .sample_index = leg.segments[0].trajectory.samples.size() - 1,
            .grasp =
              grasp_t {
                .frame_from_grasp = tool_pose_at(problem.robot, q_pick),
                .opening = candidate.grasp.opening,
              },
          },
          grasp_event_t {
            .event = grasp_event_e::release,
            .segment_index = 2,
            .sample_index = leg.segments[2].trajectory.samples.size() - 1,
            .grasp =
              grasp_t {
                .frame_from_grasp = tool_pose_at(problem.robot, q_release),
                .opening = candidate.grasp.opening,
              },
          },
        };
        leg.feasible = true;
        leg.status = solve_status_e::success;
        return leg;
      }
      return last;
    }

    regrasp_leg_t solve_place_leg(
      regrasp_problem_t const& problem, phase_scene_t const& handoff,
      grasp_candidate_t const& candidate, regrasp_config_t const& config) {
      regrasp_leg_t last;
      BodyInstance const& handoff_target = handoff.scene.body(handoff.target);
      BodyInstance const& place_target =
        problem.place.scene.body(problem.place.target);
      pose_t const body_from_grasp = compose(
        inverse(place_target.frameFromBody()),
        candidate.grasp.frame_from_grasp);
      pose_t const grasp_handoff =
        compose(handoff_target.frameFromBody(), body_from_grasp);
      pose_t const grasp_place = candidate.grasp.frame_from_grasp;
      Vector3 const approach_axis = config.approach_dir_tool.normalized();

      for (Scalar scale : std::array<Scalar, 4> {1.0, 0.75, 0.5, 0.25}) {
        pose_t approach_handoff = grasp_handoff;
        approach_handoff.position += scale * config.approach_distance *
          transform_vector(grasp_handoff, approach_axis);
        pose_t approach_place = grasp_place;
        approach_place.position += scale * config.approach_distance *
          transform_vector(grasp_place, approach_axis);

        regrasp_leg_t leg;
        leg.score = candidate.score;
        Eigen::VectorXd const q_home = problem.robot.initial_state.positions();
        motion_result_t handoff_approach = solve_free_motion(
          free_motion_problem_t {
            .scene = handoff.scene,
            .robot =
              robot_at(problem.robot, q_home, problem.robot.gripper_opening),
            .waypoints =
              {
                tool_waypoint(
                  problem.robot, approach_handoff, config.move_steps),
                tool_waypoint(problem.robot, grasp_handoff, config.grasp_steps),
              },
          },
          config.motion);
        if (handoff_approach.status != solve_status_e::success) {
          leg.status = handoff_approach.status;
          leg.failure =
            stage_failure(planning_stage_e::handoff_approach, handoff_approach);
          last = std::move(leg);
          continue;
        }
        set_static_target(
          handoff_approach.trajectory, handoff_target.frameFromBody());
        Eigen::VectorXd const q_handoff = endpoint(handoff_approach);
        attachment_t const handoff_attachment =
          attachment_at(handoff, problem.robot, q_handoff);

        motion_result_t handoff_pick = solve_grasped_motion(
          grasped_motion_problem_t {
            .scene = handoff.scene,
            .robot =
              robot_at(problem.robot, q_handoff, candidate.grasp.opening),
            .attachment = handoff_attachment,
            .waypoints = {tool_waypoint(
              problem.robot, approach_handoff, config.grasp_steps)},
          },
          config.motion);
        if (handoff_pick.status != solve_status_e::success) {
          leg.status = handoff_pick.status;
          leg.failure =
            stage_failure(planning_stage_e::handoff_pick, handoff_pick);
          last = std::move(leg);
          continue;
        }

        attachment_t place_attachment = handoff_attachment;
        place_attachment.body = problem.place.target;
        motion_result_t place_approach = solve_grasped_motion(
          grasped_motion_problem_t {
            .scene = problem.place.scene,
            .robot = robot_at(
              problem.robot, endpoint(handoff_pick), candidate.grasp.opening),
            .attachment = place_attachment,
            .waypoints =
              {
                tool_waypoint(problem.robot, approach_place, config.move_steps),
                tool_waypoint(problem.robot, grasp_place, config.grasp_steps),
              },
          },
          config.motion);
        std::optional<pose_t> const released_target =
          terminal_target(place_approach);
        if (
          place_approach.status != solve_status_e::success ||
          !released_target.has_value() ||
          !pose_close(*released_target, place_target.frameFromBody(), config)) {
          leg.status = place_approach.status == solve_status_e::success
            ? solve_status_e::infeasible
            : place_approach.status;
          leg.failure =
            stage_failure(planning_stage_e::place_approach, place_approach);
          last = std::move(leg);
          continue;
        }
        Eigen::VectorXd const q_place = endpoint(place_approach);

        motion_result_t place_retreat = solve_free_motion(
          free_motion_problem_t {
            .scene = problem.place.scene,
            .robot =
              robot_at(problem.robot, q_place, problem.robot.gripper_opening),
            .waypoints =
              {
                tool_waypoint(
                  problem.robot, approach_place, config.grasp_steps),
                joint_waypoint(q_home, config.move_steps),
              },
          },
          config.motion);
        if (place_retreat.status != solve_status_e::success) {
          leg.status = place_retreat.status;
          leg.failure =
            stage_failure(planning_stage_e::place_retreat, place_retreat);
          last = std::move(leg);
          continue;
        }
        set_static_target(
          place_retreat.trajectory, place_target.frameFromBody());

        leg.segments = {
          plan_segment_t {
            .stage = planning_stage_e::handoff_approach,
            .mode = motion_mode_e::free,
            .trajectory = std::move(handoff_approach.trajectory),
          },
          plan_segment_t {
            .stage = planning_stage_e::handoff_pick,
            .mode = motion_mode_e::attached,
            .trajectory = std::move(handoff_pick.trajectory),
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
          },
        };
        leg.events = {
          grasp_event_t {
            .event = grasp_event_e::acquire,
            .segment_index = 0,
            .sample_index = leg.segments[0].trajectory.samples.size() - 1,
            .grasp =
              grasp_t {
                .frame_from_grasp = tool_pose_at(problem.robot, q_handoff),
                .opening = candidate.grasp.opening,
              },
          },
          grasp_event_t {
            .event = grasp_event_e::release,
            .segment_index = 2,
            .sample_index = leg.segments[2].trajectory.samples.size() - 1,
            .grasp =
              grasp_t {
                .frame_from_grasp = tool_pose_at(problem.robot, q_place),
                .opening = candidate.grasp.opening,
              },
          },
        };
        leg.feasible = true;
        leg.status = solve_status_e::success;
        return leg;
      }
      return last;
    }

    bool valid_phase(phase_scene_t const& phase) {
      return phase.target.valid() &&
        phase.scene.findBody(phase.target) != nullptr;
    }

    bool valid_grasp_candidate(grasp_candidate_t const& candidate) {
      return is_valid(candidate.grasp.frame_from_grasp) &&
        std::isfinite(candidate.grasp.opening) &&
        std::isfinite(candidate.score);
    }

  }  // namespace

  regrasp_pose_result_t generate_regrasp_poses(
    regrasp_pose_problem_t const& problem, regrasp_config_t const& config) {
    if (!valid_config(config)) {
      return invalid_pose_result(
        "invalid_config", "regrasp configuration is invalid");
    }
    if (
      problem.body_model == nullptr ||
      !is_valid(problem.frame_from_body_pick) ||
      !is_valid(problem.frame_from_body_place) ||
      !problem.handoff_position.allFinite()) {
      return invalid_pose_result(
        "invalid_pose_problem",
        "regrasp pose generation requires a body and finite poses");
    }

    stable_pose_result_t const stable = solve_stable_poses(
      stable_pose_problem_t {
        .body_model = problem.body_model,
        .com_offset_body = Vector3::Zero(),
      },
      config.stable_pose);
    if (stable.status != solve_status_e::success) {
      return regrasp_pose_result_t {
        .status = stable.status,
        .poses = {},
        .solver = stable.solver,
        .failure = stable.failure,
      };
    }

    regrasp_pose_result_t result {
      .status = solve_status_e::success,
      .poses = {},
      .solver = stable.solver,
      .failure = {},
    };
    result.poses.reserve(
      stable.poses.size() * static_cast<std::size_t>(config.yaw_samples));
    for (stable_pose_t const& pose : stable.poses) {
      for (int i = 0; i < config.yaw_samples; ++i) {
        Scalar const yaw = 2.0 * std::numbers::pi * i / config.yaw_samples;
        pose_t const frame_from_body = resting_pose(
          *problem.body_model, pose, stable.reference_point_body,
          problem.handoff_position, yaw);
        if (
          frame_from_body.orientation.angularDistance(
            problem.frame_from_body_pick.orientation) >
            config.max_handoff_orientation_distance ||
          frame_from_body.orientation.angularDistance(
            problem.frame_from_body_place.orientation) >
            config.max_handoff_orientation_distance) {
          continue;
        }
        result.poses.push_back(regrasp_pose_t {
          .stable_pose = pose,
          .frame_from_body = frame_from_body,
          .yaw = yaw,
        });
      }
    }
    if (result.poses.empty()) {
      result.status = solve_status_e::infeasible;
      result.failure = planner_failure_t {
        .code = "no_handoff_pose",
        .message =
          "no stable handoff orientation passed the pick/place rotation gate",
        .retryable = false,
      };
    }
    return result;
  }

  plan_result_t solve_regrasp(
    regrasp_problem_t const& problem, regrasp_config_t const& config) {
    if (!valid_config(config)) {
      return invalid_plan_result(
        "invalid_config", "regrasp configuration is invalid");
    }
    if (
      !valid_phase(problem.pick) || !valid_phase(problem.handoff) ||
      !valid_phase(problem.place)) {
      return invalid_plan_result(
        "invalid_phase", "every regrasp phase must contain its target");
    }
    if (
      problem.pick.scene.frame() != problem.handoff.scene.frame() ||
      problem.pick.scene.frame() != problem.place.scene.frame() ||
      problem.pick.scene.frame() != problem.robot.initial_state.frame()) {
      return invalid_plan_result(
        "frame_mismatch", "regrasp phases and robot must share one frame");
    }
    BodyInstance const& pick_target =
      problem.pick.scene.body(problem.pick.target);
    BodyInstance const& handoff_target =
      problem.handoff.scene.body(problem.handoff.target);
    BodyInstance const& place_target =
      problem.place.scene.body(problem.place.target);
    if (
      pick_target.model().id() != handoff_target.model().id() ||
      pick_target.model().id() != place_target.model().id()) {
      return invalid_plan_result(
        "target_model_mismatch",
        "regrasp phase targets must represent the same body model");
    }
    if (problem.pick_grasps.empty() || problem.place_grasps.empty()) {
      return invalid_plan_result(
        "missing_grasps",
        "regrasp planning requires pick-side and place-side grasps");
    }
    if (
      !std::all_of(
        problem.pick_grasps.begin(), problem.pick_grasps.end(),
        valid_grasp_candidate) ||
      !std::all_of(
        problem.place_grasps.begin(), problem.place_grasps.end(),
        valid_grasp_candidate)) {
      return invalid_plan_result(
        "invalid_grasp", "regrasp candidates must be finite");
    }

    planner_clock_t::time_point const total_start = planner_clock_t::now();
    planner_clock_t::time_point const pose_start = planner_clock_t::now();
    regrasp_pose_result_t const poses = generate_regrasp_poses(
      regrasp_pose_problem_t {
        .body_model = pick_target.modelPtr(),
        .frame_from_body_pick = pick_target.frameFromBody(),
        .frame_from_body_place = place_target.frameFromBody(),
        .handoff_position = problem.handoff_position,
      },
      config);
    if (poses.status != solve_status_e::success) {
      plan_result_t result {
        .status = poses.status,
        .candidates = {},
        .selected_index = std::nullopt,
        .failure = poses.failure,
        .timings = {},
      };
      result.timings.grasp_generation_seconds = elapsed_seconds(pose_start);
      result.timings.total_seconds = elapsed_seconds(total_start);
      return result;
    }

    plan_result_t result {
      .status = solve_status_e::infeasible,
      .candidates = {},
      .selected_index = std::nullopt,
      .failure =
        {
          .code = "regrasp_motion_infeasible",
          .message = "no stable pose and grasp pair produced a feasible motion",
          .retryable = true,
        },
      .timings = {},
    };
    result.timings.grasp_generation_seconds = elapsed_seconds(pose_start);
    result.timings.grasp_candidates =
      problem.pick_grasps.size() + problem.place_grasps.size();
    planner_clock_t::time_point const motion_start = planner_clock_t::now();
    for (regrasp_pose_t const& pose : poses.poses) {
      phase_scene_t const handoff =
        phase_with_target_pose(problem.handoff, pose.frame_from_body);
      std::vector<regrasp_leg_t> pick_legs(problem.pick_grasps.size());
      std::vector<regrasp_leg_t> place_legs(problem.place_grasps.size());
      std::size_t const pick_count = problem.pick_grasps.size();
      detail::planner_parallel_for(
        pick_count + problem.place_grasps.size(), config.worker_count,
        [&](std::size_t i) {
          if (i < pick_count) {
            pick_legs[i] =
              solve_pick_leg(problem, handoff, problem.pick_grasps[i], config);
          } else {
            std::size_t const place_index = i - pick_count;
            place_legs[place_index] = solve_place_leg(
              problem, handoff, problem.place_grasps[place_index], config);
          }
        });
      result.timings.motion_candidates += pick_legs.size() + place_legs.size();

      for (regrasp_leg_t const& leg : pick_legs) {
        if (leg.status == solve_status_e::invalid_problem) {
          result.status = solve_status_e::invalid_problem;
          result.candidates.clear();
          result.selected_index = std::nullopt;
          result.failure = leg.failure;
          result.timings.trajectory_optimization_seconds =
            elapsed_seconds(motion_start);
          result.timings.total_seconds = elapsed_seconds(total_start);
          return result;
        }
        if (
          !leg.feasible && result.failure.code == "regrasp_motion_infeasible" &&
          !leg.failure.code.empty()) {
          result.failure = leg.failure;
        }
      }
      for (regrasp_leg_t const& leg : place_legs) {
        if (leg.status == solve_status_e::invalid_problem) {
          result.status = solve_status_e::invalid_problem;
          result.candidates.clear();
          result.selected_index = std::nullopt;
          result.failure = leg.failure;
          result.timings.trajectory_optimization_seconds =
            elapsed_seconds(motion_start);
          result.timings.total_seconds = elapsed_seconds(total_start);
          return result;
        }
        if (
          !leg.feasible && result.failure.code == "regrasp_motion_infeasible" &&
          !leg.failure.code.empty()) {
          result.failure = leg.failure;
        }
      }

      std::vector<std::pair<std::size_t, std::size_t>> pairs;
      for (std::size_t i = 0; i < pick_legs.size(); ++i) {
        for (std::size_t j = 0; j < place_legs.size(); ++j) {
          if (pick_legs[i].feasible && place_legs[j].feasible) {
            pairs.emplace_back(i, j);
          }
        }
      }
      std::sort(pairs.begin(), pairs.end(), [&](auto const& a, auto const& b) {
        return pick_legs[a.first].score + place_legs[a.second].score >
          pick_legs[b.first].score + place_legs[b.second].score;
      });
      for (auto const& [pick_index, place_index] : pairs) {
        regrasp_leg_t& pick_leg = pick_legs[pick_index];
        regrasp_leg_t& place_leg = place_legs[place_index];
        plan_candidate_t candidate;
        candidate.score = pick_leg.score + place_leg.score;
        candidate.segments = pick_leg.segments;
        candidate.segments.insert(
          candidate.segments.end(), place_leg.segments.begin(),
          place_leg.segments.end());
        candidate.grasp_events = pick_leg.events;
        for (grasp_event_t event : place_leg.events) {
          event.segment_index += pick_leg.segments.size();
          candidate.grasp_events.push_back(std::move(event));
        }
        result.candidates.push_back(std::move(candidate));
        if (
          result.candidates.size() >=
          static_cast<std::size_t>(config.max_candidates)) {
          result.status = solve_status_e::success;
          result.selected_index = 0;
          result.failure = {};
          result.timings.trajectory_optimization_seconds =
            elapsed_seconds(motion_start);
          result.timings.total_seconds = elapsed_seconds(total_start);
          return result;
        }
      }
    }
    result.timings.trajectory_optimization_seconds =
      elapsed_seconds(motion_start);
    result.timings.total_seconds = elapsed_seconds(total_start);
    return result;
  }

}  // namespace stacking_core
