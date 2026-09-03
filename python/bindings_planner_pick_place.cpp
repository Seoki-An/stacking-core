#include "bindings_common.hpp"

#include <stacking_core/planner/pick_place.hpp>

#include <nanobind/stl/vector.h>

#include <utility>
#include <vector>

namespace nb = nanobind;

namespace stacking_core::python {

void bind_planner_pick_place(nb::module_ &module) {
  nb::class_<direct_plan_config_t>(module, "DirectPlanConfig")
      .def(nb::init<>())
      .def_rw("grasp_sampling", &direct_plan_config_t::grasp_sampling)
      .def_rw("grasp_generation", &direct_plan_config_t::grasp_generation)
      .def_rw("inverse_kinematics", &direct_plan_config_t::inverse_kinematics)
      .def_rw("grasp_simulation", &direct_plan_config_t::grasp_simulation)
      .def_rw("motion", &direct_plan_config_t::motion)
      .def_rw("simulation_refinement",
              &direct_plan_config_t::simulation_refinement)
      .def_rw("approach_dir_tool", &direct_plan_config_t::approach_dir_tool)
      .def_rw("approach_distance", &direct_plan_config_t::approach_distance)
      .def_rw("target_pos_tol", &direct_plan_config_t::target_pos_tol)
      .def_rw("target_rot_tol", &direct_plan_config_t::target_rot_tol)
      .def_rw("move_steps", &direct_plan_config_t::move_steps)
      .def_rw("grasp_steps", &direct_plan_config_t::grasp_steps)
      .def_rw("max_candidates", &direct_plan_config_t::max_candidates)
      .def_rw("worker_count", &direct_plan_config_t::worker_count);

  nb::class_<inhand_plan_config_t>(module, "InhandPlanConfig")
      .def(nb::init<>())
      .def_rw("motion", &inhand_plan_config_t::motion)
      .def_rw("approach_dir_tool", &inhand_plan_config_t::approach_dir_tool)
      .def_rw("approach_distance", &inhand_plan_config_t::approach_distance)
      .def_rw("target_pos_tol", &inhand_plan_config_t::target_pos_tol)
      .def_rw("target_rot_tol", &inhand_plan_config_t::target_rot_tol)
      .def_rw("move_steps", &inhand_plan_config_t::move_steps)
      .def_rw("grasp_steps", &inhand_plan_config_t::grasp_steps);

  nb::class_<regrasp_config_t>(module, "RegraspConfig")
      .def(nb::init<>())
      .def_rw("stable_pose", &regrasp_config_t::stable_pose)
      .def_rw("motion", &regrasp_config_t::motion)
      .def_rw("approach_dir_tool", &regrasp_config_t::approach_dir_tool)
      .def_rw("approach_distance", &regrasp_config_t::approach_distance)
      .def_rw("target_pos_tol", &regrasp_config_t::target_pos_tol)
      .def_rw("target_rot_tol", &regrasp_config_t::target_rot_tol)
      .def_rw("max_handoff_orientation_distance",
              &regrasp_config_t::max_handoff_orientation_distance)
      .def_rw("move_steps", &regrasp_config_t::move_steps)
      .def_rw("grasp_steps", &regrasp_config_t::grasp_steps)
      .def_rw("yaw_samples", &regrasp_config_t::yaw_samples)
      .def_rw("max_candidates", &regrasp_config_t::max_candidates)
      .def_rw("worker_count", &regrasp_config_t::worker_count);

  nb::class_<pick_place_config_t>(module, "PickPlaceConfig")
      .def(nb::init<>())
      .def_rw("direct", &pick_place_config_t::direct)
      .def_rw("regrasp", &pick_place_config_t::regrasp)
      .def_rw("allow_regrasp", &pick_place_config_t::allow_regrasp);

  nb::class_<direct_plan_problem_t>(module, "DirectPlanProblem")
      .def(nb::new_([](phase_scene_t pick, phase_scene_t place,
                       motion_robot_t robot, gripper_model_t gripper,
                       std::vector<joint_grasp_candidate_t> grasp_candidates) {
             return direct_plan_problem_t{
                 .pick = std::move(pick),
                 .place = std::move(place),
                 .robot = std::move(robot),
                 .gripper = std::move(gripper),
                 .grasp_candidates = std::move(grasp_candidates),
             };
           }),
           nb::arg("pick"), nb::arg("place"), nb::arg("robot"),
           nb::arg("gripper"),
           nb::arg("grasp_candidates") = std::vector<joint_grasp_candidate_t>{})
      .def_rw("pick", &direct_plan_problem_t::pick)
      .def_rw("place", &direct_plan_problem_t::place)
      .def_rw("robot", &direct_plan_problem_t::robot)
      .def_rw("gripper", &direct_plan_problem_t::gripper)
      .def_rw("grasp_candidates", &direct_plan_problem_t::grasp_candidates);

  nb::class_<inhand_plan_problem_t>(module, "InhandPlanProblem")
      .def(nb::new_([](phase_scene_t place, motion_robot_t robot,
                       attachment_t attachment) {
             return inhand_plan_problem_t{
                 .place = std::move(place),
                 .robot = std::move(robot),
                 .attachment = std::move(attachment),
             };
           }),
           nb::arg("place"), nb::arg("robot"), nb::arg("attachment"))
      .def_rw("place", &inhand_plan_problem_t::place)
      .def_rw("robot", &inhand_plan_problem_t::robot)
      .def_rw("attachment", &inhand_plan_problem_t::attachment);

  nb::class_<regrasp_problem_t>(module, "RegraspProblem")
      .def(nb::new_([](phase_scene_t pick, phase_scene_t handoff,
                       phase_scene_t place, motion_robot_t robot,
                       std::vector<grasp_candidate_t> pick_grasps,
                       std::vector<grasp_candidate_t> place_grasps,
                       Vector3 handoff_position) {
             return regrasp_problem_t{
                 .pick = std::move(pick),
                 .handoff = std::move(handoff),
                 .place = std::move(place),
                 .robot = std::move(robot),
                 .pick_grasps = std::move(pick_grasps),
                 .place_grasps = std::move(place_grasps),
                 .handoff_position = std::move(handoff_position),
             };
           }),
           nb::arg("pick"), nb::arg("handoff"), nb::arg("place"),
           nb::arg("robot"), nb::arg("pick_grasps"), nb::arg("place_grasps"),
           nb::arg("handoff_position"))
      .def_rw("pick", &regrasp_problem_t::pick)
      .def_rw("handoff", &regrasp_problem_t::handoff)
      .def_rw("place", &regrasp_problem_t::place)
      .def_rw("robot", &regrasp_problem_t::robot)
      .def_rw("pick_grasps", &regrasp_problem_t::pick_grasps)
      .def_rw("place_grasps", &regrasp_problem_t::place_grasps)
      .def_rw("handoff_position", &regrasp_problem_t::handoff_position);

  nb::class_<pick_place_problem_t>(module, "PickPlaceProblem")
      .def(nb::new_([](direct_plan_problem_t direct, phase_scene_t handoff,
                       Vector3 handoff_position,
                       std::vector<grasp_candidate_t> pick_grasp_candidates,
                       std::vector<grasp_candidate_t> place_grasp_candidates) {
             return pick_place_problem_t{
                 .direct = std::move(direct),
                 .handoff = std::move(handoff),
                 .handoff_position = std::move(handoff_position),
                 .pick_grasp_candidates = std::move(pick_grasp_candidates),
                 .place_grasp_candidates = std::move(place_grasp_candidates),
             };
           }),
           nb::arg("direct"), nb::arg("handoff"), nb::arg("handoff_position"),
           nb::arg("pick_grasp_candidates") = std::vector<grasp_candidate_t>{},
           nb::arg("place_grasp_candidates") = std::vector<grasp_candidate_t>{})
      .def_rw("direct", &pick_place_problem_t::direct)
      .def_rw("handoff", &pick_place_problem_t::handoff)
      .def_rw("handoff_position", &pick_place_problem_t::handoff_position)
      .def_rw("pick_grasp_candidates",
              &pick_place_problem_t::pick_grasp_candidates)
      .def_rw("place_grasp_candidates",
              &pick_place_problem_t::place_grasp_candidates);

  module.def(
      "solve_direct",
      [](direct_plan_problem_t const &problem,
         direct_plan_config_t const &config) {
        nb::gil_scoped_release release;
        return solve_direct(problem, config);
      },
      nb::arg("problem"), nb::arg("config") = direct_plan_config_t{});

  module.def(
      "solve_inhand",
      [](inhand_plan_problem_t const &problem,
         inhand_plan_config_t const &config) {
        nb::gil_scoped_release release;
        return solve_inhand(problem, config);
      },
      nb::arg("problem"), nb::arg("config") = inhand_plan_config_t{});

  module.def(
      "solve_regrasp",
      [](regrasp_problem_t const &problem, regrasp_config_t const &config) {
        nb::gil_scoped_release release;
        return solve_regrasp(problem, config);
      },
      nb::arg("problem"), nb::arg("config") = regrasp_config_t{});

  module.def(
      "solve_pick_place",
      [](pick_place_problem_t const &problem,
         pick_place_config_t const &config) {
        nb::gil_scoped_release release;
        return solve_pick_place(problem, config);
      },
      nb::arg("problem"), nb::arg("config") = pick_place_config_t{});
}

} // namespace stacking_core::python
