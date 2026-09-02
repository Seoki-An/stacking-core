#include "bindings_common.hpp"

#include <stacking_core/planner/inverse_kinematics.hpp>
#include <stacking_core/planner/motion.hpp>

#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace nb = nanobind;

namespace stacking_core::python {

void bind_planner_motion(nb::module_ &module) {
  nb::enum_<inverse_kinematics_initialization_e>(
      module, "InverseKinematicsInitialization")
      .value("SWING", inverse_kinematics_initialization_e::swing)
      .value("PROVIDED", inverse_kinematics_initialization_e::provided);

  nb::class_<inverse_kinematics_config_t>(module, "InverseKinematicsConfig")
      .def(nb::init<>())
      .def_rw("max_iters", &inverse_kinematics_config_t::max_iters)
      .def_rw("tol", &inverse_kinematics_config_t::tol)
      .def_rw("initialization", &inverse_kinematics_config_t::initialization);

  nb::class_<motion_planning_config_t>(module, "MotionPlanningConfig")
      .def(nb::init<>())
      .def_rw("smooth_weight", &motion_planning_config_t::smooth_weight)
      .def_rw("boundary_weight", &motion_planning_config_t::boundary_weight)
      .def_rw("collision_weight", &motion_planning_config_t::collision_weight)
      .def_rw("joint_limit_weight",
              &motion_planning_config_t::joint_limit_weight)
      .def_rw("smooth_boundary_alpha",
              &motion_planning_config_t::smooth_boundary_alpha)
      .def_rw("swing_smoothness_scale",
              &motion_planning_config_t::swing_smoothness_scale)
      .def_rw("collision_margin", &motion_planning_config_t::collision_margin)
      .def_rw("plane_feasibility_margin",
              &motion_planning_config_t::plane_feasibility_margin)
      .def_rw("joint_limit_margin",
              &motion_planning_config_t::joint_limit_margin)
      .def_rw("grasped_boundary_pos_scale",
              &motion_planning_config_t::grasped_boundary_pos_scale)
      .def_rw("grasped_boundary_rot_scale",
              &motion_planning_config_t::grasped_boundary_rot_scale)
      .def_rw("target_collision_tol",
              &motion_planning_config_t::target_collision_tol)
      .def_rw("step_size", &motion_planning_config_t::step_size)
      .def_rw("max_iters", &motion_planning_config_t::max_iters)
      .def_rw("tol", &motion_planning_config_t::tol)
      .def_rw("ik_max_iters", &motion_planning_config_t::ik_max_iters)
      .def_rw("ik_tol", &motion_planning_config_t::ik_tol);

  nb::class_<collision_body_pair_t>(module, "CollisionBodyPair")
      .def(nb::new_([](std::uint64_t first, std::uint64_t second) {
             return collision_body_pair_t{
                 .first = EntityId{first},
                 .second = EntityId{second},
             };
           }),
           nb::arg("first"), nb::arg("second"))
      .def_prop_ro(
          "first",
          [](collision_body_pair_t const &pair) { return pair.first.value(); })
      .def_prop_ro("second", [](collision_body_pair_t const &pair) {
        return pair.second.value();
      });

  nb::class_<motion_link_body_t>(module, "MotionLinkBody")
      .def(nb::new_([](std::uint64_t link, std::uint64_t entity,
                       std::shared_ptr<BodyModel> body_model) {
             return motion_link_body_t{
                 .link = LinkId{link},
                 .entity = EntityId{entity},
                 .body_model = std::move(body_model),
             };
           }),
           nb::arg("link"), nb::arg("entity"), nb::arg("body_model"))
      .def_prop_ro(
          "link",
          [](motion_link_body_t const &body) { return body.link.value(); })
      .def_prop_ro(
          "entity",
          [](motion_link_body_t const &body) { return body.entity.value(); })
      .def_ro("body_model", &motion_link_body_t::body_model);

  nb::class_<motion_robot_t>(module, "MotionRobot")
      .def(
          nb::new_([](KinematicState initial_state, std::uint64_t tool_link,
                      array_t const &link_from_tool, Scalar gripper_opening,
                      std::vector<motion_link_body_t> collision_bodies,
                      std::vector<collision_body_pair_t> self_collision_pairs) {
            return motion_robot_t{
                .initial_state = std::move(initial_state),
                .tool_link = LinkId{tool_link},
                .link_from_tool =
                    pose_from_array(link_from_tool, "link_from_tool"),
                .gripper_opening = gripper_opening,
                .ik_initializer = {},
                .collision_bodies = std::move(collision_bodies),
                .self_collision_pairs = std::move(self_collision_pairs),
            };
          }),
          nb::arg("initial_state"), nb::arg("tool_link"),
          nb::arg("link_from_tool"), nb::arg("gripper_opening") = 0.0,
          nb::arg("collision_bodies") = std::vector<motion_link_body_t>{},
          nb::arg("self_collision_pairs") =
              std::vector<collision_body_pair_t>{})
      .def_rw("initial_state", &motion_robot_t::initial_state)
      .def_prop_ro(
          "tool_link",
          [](motion_robot_t const &robot) { return robot.tool_link.value(); })
      .def_prop_rw(
          "link_from_tool",
          [](motion_robot_t const &robot) {
            return pose_vector(robot.link_from_tool);
          },
          [](motion_robot_t &robot, array_t const &value) {
            robot.link_from_tool = pose_from_array(value, "link_from_tool");
          })
      .def_rw("gripper_opening", &motion_robot_t::gripper_opening)
      .def_rw("collision_bodies", &motion_robot_t::collision_bodies)
      .def_rw("self_collision_pairs", &motion_robot_t::self_collision_pairs);

  nb::class_<inverse_kinematics_problem_t>(module, "InverseKinematicsProblem")
      .def(nb::new_([](KinematicState initial_state, std::uint64_t link,
                       array_t const &frame_from_link, bool position_only) {
             return inverse_kinematics_problem_t{
                 .initial_state = std::move(initial_state),
                 .link = LinkId{link},
                 .frame_from_link =
                     pose_from_array(frame_from_link, "frame_from_link"),
                 .position_only = position_only,
             };
           }),
           nb::arg("initial_state"), nb::arg("link"),
           nb::arg("frame_from_link"), nb::arg("position_only") = false)
      .def_rw("initial_state", &inverse_kinematics_problem_t::initial_state)
      .def_prop_ro("link",
                   [](inverse_kinematics_problem_t const &problem) {
                     return problem.link.value();
                   })
      .def_prop_rw(
          "frame_from_link",
          [](inverse_kinematics_problem_t const &problem) {
            return pose_vector(problem.frame_from_link);
          },
          [](inverse_kinematics_problem_t &problem, array_t const &value) {
            problem.frame_from_link = pose_from_array(value, "frame_from_link");
          })
      .def_rw("position_only", &inverse_kinematics_problem_t::position_only);

  nb::class_<inverse_kinematics_result_t>(module, "InverseKinematicsResult")
      .def_prop_ro("status",
                   [](inverse_kinematics_result_t const &result) {
                     return result.status;
                   })
      .def_prop_ro("positions",
                   [](inverse_kinematics_result_t const &result) {
                     return result.positions;
                   })
      .def_prop_ro("frame_from_link",
                   [](inverse_kinematics_result_t const &result) {
                     return pose_vector(result.frame_from_link);
                   })
      .def_ro("pos_error", &inverse_kinematics_result_t::pos_error)
      .def_ro("rot_error", &inverse_kinematics_result_t::rot_error)
      .def_ro("solver", &inverse_kinematics_result_t::solver)
      .def_ro("failure", &inverse_kinematics_result_t::failure);

  nb::class_<joint_goal_t>(module, "JointGoal")
      .def(nb::new_([](Eigen::VectorXd positions, bool preserve_branch) {
             return joint_goal_t{
                 .positions = std::move(positions),
                 .preserve_branch = preserve_branch,
             };
           }),
           nb::arg("positions"), nb::arg("preserve_branch") = false)
      .def_rw("positions", &joint_goal_t::positions)
      .def_rw("preserve_branch", &joint_goal_t::preserve_branch);

  nb::class_<link_pose_goal_t>(module, "LinkPoseGoal")
      .def(nb::new_([](std::uint64_t link, array_t const &frame_from_link) {
             return link_pose_goal_t{
                 .link = LinkId{link},
                 .frame_from_link =
                     pose_from_array(frame_from_link, "frame_from_link"),
             };
           }),
           nb::arg("link"), nb::arg("frame_from_link"))
      .def_prop_ro(
          "link",
          [](link_pose_goal_t const &goal) { return goal.link.value(); })
      .def_prop_rw(
          "frame_from_link",
          [](link_pose_goal_t const &goal) {
            return pose_vector(goal.frame_from_link);
          },
          [](link_pose_goal_t &goal, array_t const &value) {
            goal.frame_from_link = pose_from_array(value, "frame_from_link");
          });

  nb::class_<motion_waypoint_t>(module, "MotionWaypoint")
      .def(nb::new_([](joint_goal_t goal, int steps) {
             return motion_waypoint_t{.goal = std::move(goal), .steps = steps};
           }),
           nb::arg("goal"), nb::arg("steps") = 0)
      .def(nb::new_([](link_pose_goal_t goal, int steps) {
             return motion_waypoint_t{.goal = std::move(goal), .steps = steps};
           }),
           nb::arg("goal"), nb::arg("steps") = 0)
      .def_rw("goal", &motion_waypoint_t::goal)
      .def_rw("steps", &motion_waypoint_t::steps);

  nb::class_<free_motion_problem_t>(module, "FreeMotionProblem")
      .def(nb::new_([](SceneView scene, motion_robot_t robot,
                       std::vector<motion_waypoint_t> waypoints) {
             return free_motion_problem_t{
                 .scene = std::move(scene),
                 .robot = std::move(robot),
                 .waypoints = std::move(waypoints),
             };
           }),
           nb::arg("scene"), nb::arg("robot"), nb::arg("waypoints"))
      .def_rw("scene", &free_motion_problem_t::scene)
      .def_rw("robot", &free_motion_problem_t::robot)
      .def_rw("waypoints", &free_motion_problem_t::waypoints);

  nb::class_<grasped_motion_problem_t>(module, "GraspedMotionProblem")
      .def(nb::new_([](SceneView scene, motion_robot_t robot,
                       attachment_t attachment,
                       std::vector<motion_waypoint_t> waypoints) {
             return grasped_motion_problem_t{
                 .scene = std::move(scene),
                 .robot = std::move(robot),
                 .attachment = std::move(attachment),
                 .waypoints = std::move(waypoints),
             };
           }),
           nb::arg("scene"), nb::arg("robot"), nb::arg("attachment"),
           nb::arg("waypoints"))
      .def_rw("scene", &grasped_motion_problem_t::scene)
      .def_rw("robot", &grasped_motion_problem_t::robot)
      .def_rw("attachment", &grasped_motion_problem_t::attachment)
      .def_rw("waypoints", &grasped_motion_problem_t::waypoints);

  nb::class_<motion_result_t>(module, "MotionResult")
      .def_prop_ro("status",
                   [](motion_result_t const &result) { return result.status; })
      .def_ro("trajectory", &motion_result_t::trajectory)
      .def_ro("solver", &motion_result_t::solver)
      .def_ro("failure", &motion_result_t::failure);

  module.def(
      "solve_inverse_kinematics",
      [](inverse_kinematics_problem_t const &problem,
         inverse_kinematics_config_t const &config) {
        nb::gil_scoped_release release;
        return solve_inverse_kinematics(problem, config);
      },
      nb::arg("problem"), nb::arg("config") = inverse_kinematics_config_t{});

  module.def(
      "solve_free_motion",
      [](free_motion_problem_t const &problem,
         motion_planning_config_t const &config) {
        nb::gil_scoped_release release;
        return solve_free_motion(problem, config);
      },
      nb::arg("problem"), nb::arg("config") = motion_planning_config_t{});

  module.def(
      "solve_grasped_motion",
      [](grasped_motion_problem_t const &problem,
         motion_planning_config_t const &config) {
        nb::gil_scoped_release release;
        return solve_grasped_motion(problem, config);
      },
      nb::arg("problem"), nb::arg("config") = motion_planning_config_t{});
}

} // namespace stacking_core::python
