#include "bindings_common.hpp"

#include <stacking_core/collision/types.hpp>
#include <stacking_core/planner/manipulation.hpp>

#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <cstdint>
#include <optional>
#include <utility>

namespace nb = nanobind;

namespace stacking_core::python {

void bind_planner_types(nb::module_ &module) {
  nb::enum_<solve_status_e>(module, "SolveStatus")
      .value("SUCCESS", solve_status_e::success)
      .value("INFEASIBLE", solve_status_e::infeasible)
      .value("MAX_ITERS", solve_status_e::max_iters)
      .value("CANCELLED", solve_status_e::cancelled)
      .value("INVALID_PROBLEM", solve_status_e::invalid_problem);

  nb::enum_<planning_stage_e>(module, "PlanningStage")
      .value("PICK_APPROACH", planning_stage_e::pick_approach)
      .value("PICK_RETREAT", planning_stage_e::pick_retreat)
      .value("TRANSFER", planning_stage_e::transfer)
      .value("HANDOFF_PLACE", planning_stage_e::handoff_place)
      .value("HANDOFF_RETREAT", planning_stage_e::handoff_retreat)
      .value("HANDOFF_APPROACH", planning_stage_e::handoff_approach)
      .value("HANDOFF_PICK", planning_stage_e::handoff_pick)
      .value("PLACE_APPROACH", planning_stage_e::place_approach)
      .value("PLACE_RETREAT", planning_stage_e::place_retreat);

  nb::enum_<motion_mode_e>(module, "MotionMode")
      .value("FREE", motion_mode_e::free)
      .value("ATTACHED", motion_mode_e::attached);

  nb::enum_<grasp_event_e>(module, "GraspEventType")
      .value("ACQUIRE", grasp_event_e::acquire)
      .value("RELEASE", grasp_event_e::release);

  nb::class_<planner_failure_t>(module, "PlannerFailure")
      .def(nb::init<>())
      .def_rw("code", &planner_failure_t::code)
      .def_rw("message", &planner_failure_t::message)
      .def_rw("retryable", &planner_failure_t::retryable);

  nb::class_<planner_solver_stats_t>(module, "PlannerSolverStats")
      .def(nb::init<>())
      .def_rw("iters", &planner_solver_stats_t::iters)
      .def_rw("converged", &planner_solver_stats_t::converged)
      .def_rw("objective", &planner_solver_stats_t::objective)
      .def_rw("grad_norm", &planner_solver_stats_t::grad_norm);

  nb::class_<geometry_instance_id_t>(module, "GeometryInstanceId")
      .def(nb::new_([](std::uint64_t entity, std::uint64_t geometry) {
             return geometry_instance_id_t{
                 .entity = EntityId{entity},
                 .geometry = GeometryId{geometry},
             };
           }),
           nb::arg("entity"), nb::arg("geometry"))
      .def_prop_rw(
          "entity",
          [](geometry_instance_id_t const &id) { return id.entity.value(); },
          [](geometry_instance_id_t &id, std::uint64_t value) {
            id.entity = EntityId{value};
          })
      .def_prop_rw(
          "geometry",
          [](geometry_instance_id_t const &id) { return id.geometry.value(); },
          [](geometry_instance_id_t &id, std::uint64_t value) {
            id.geometry = GeometryId{value};
          })
      .def_prop_ro("valid", &geometry_instance_id_t::valid);

  nb::class_<collision_pair_t>(module, "CollisionPair")
      .def(nb::init<>())
      .def_rw("first", &collision_pair_t::first)
      .def_rw("second", &collision_pair_t::second);

  nb::class_<contact_feature_t>(module, "ContactFeature")
      .def(nb::init<>())
      .def_rw("pair", &contact_feature_t::pair)
      .def_rw("gap", &contact_feature_t::gap)
      .def_rw("point_first", &contact_feature_t::point_first)
      .def_rw("point_second", &contact_feature_t::point_second)
      .def_rw("normal", &contact_feature_t::normal)
      .def_prop_ro("penetrating", &contact_feature_t::penetrating);

  nb::class_<grasp_t>(module, "Grasp")
      .def(nb::init<>())
      .def(nb::new_([](array_t const &frame_from_grasp, Scalar opening) {
             return grasp_t{
                 .frame_from_grasp =
                     pose_from_array(frame_from_grasp, "frame_from_grasp"),
                 .opening = opening,
             };
           }),
           nb::arg("frame_from_grasp"), nb::arg("opening") = 0.0)
      .def_prop_rw(
          "frame_from_grasp",
          [](grasp_t const &grasp) {
            return pose_vector(grasp.frame_from_grasp);
          },
          [](grasp_t &grasp, array_t const &value) {
            grasp.frame_from_grasp = pose_from_array(value, "frame_from_grasp");
          })
      .def_rw("opening", &grasp_t::opening);

  nb::class_<robot_state_t>(module, "RobotState")
      .def(nb::init<>())
      .def(nb::new_([](Eigen::VectorXd positions, Scalar gripper_opening) {
             return robot_state_t{
                 .positions = std::move(positions),
                 .gripper_opening = gripper_opening,
             };
           }),
           nb::arg("positions"), nb::arg("gripper_opening") = 0.0)
      .def_rw("positions", &robot_state_t::positions)
      .def_rw("gripper_opening", &robot_state_t::gripper_opening);

  nb::class_<attachment_t>(module, "Attachment")
      .def(nb::new_([](std::uint64_t body, std::uint64_t link,
                       array_t const &link_from_body) {
             return attachment_t{
                 .body = EntityId{body},
                 .link = LinkId{link},
                 .link_from_body =
                     pose_from_array(link_from_body, "link_from_body"),
             };
           }),
           nb::arg("body"), nb::arg("link"), nb::arg("link_from_body"))
      .def_prop_rw(
          "body", [](attachment_t const &value) { return value.body.value(); },
          [](attachment_t &value, std::uint64_t id) {
            value.body = EntityId{id};
          })
      .def_prop_rw(
          "link", [](attachment_t const &value) { return value.link.value(); },
          [](attachment_t &value, std::uint64_t id) {
            value.link = LinkId{id};
          })
      .def_prop_rw(
          "link_from_body",
          [](attachment_t const &value) {
            return pose_vector(value.link_from_body);
          },
          [](attachment_t &value, array_t const &pose) {
            value.link_from_body = pose_from_array(pose, "link_from_body");
          });

  nb::class_<trajectory_sample_t>(module, "TrajectorySample")
      .def(nb::init<>())
      .def_rw("robot", &trajectory_sample_t::robot)
      .def_prop_rw(
          "frame_from_target",
          [](trajectory_sample_t const &sample) {
            return optional_pose_object(sample.frame_from_target);
          },
          [](trajectory_sample_t &sample, nb::object const &pose) {
            sample.frame_from_target =
                optional_pose_from_object(pose, "frame_from_target");
          });

  nb::class_<trajectory_t>(module, "Trajectory")
      .def(nb::init<>())
      .def_rw("samples", &trajectory_t::samples);

  nb::class_<grasp_contact_t>(module, "GraspContact")
      .def(nb::init<>())
      .def_rw("feature", &grasp_contact_t::feature)
      .def_rw("force", &grasp_contact_t::force);

  nb::class_<plan_segment_t>(module, "PlanSegment")
      .def(nb::init<>())
      .def_rw("stage", &plan_segment_t::stage)
      .def_rw("mode", &plan_segment_t::mode)
      .def_rw("trajectory", &plan_segment_t::trajectory);

  nb::class_<grasp_event_t>(module, "GraspEvent")
      .def(nb::init<>())
      .def_rw("event", &grasp_event_t::event)
      .def_rw("segment_index", &grasp_event_t::segment_index)
      .def_rw("sample_index", &grasp_event_t::sample_index)
      .def_rw("grasp", &grasp_event_t::grasp);

  nb::class_<plan_diagnostic_t>(module, "PlanDiagnostic")
      .def(nb::init<>())
      .def_rw("stage", &plan_diagnostic_t::stage)
      .def_rw("failure", &plan_diagnostic_t::failure);

  nb::class_<plan_candidate_t>(module, "PlanCandidate")
      .def(nb::init<>())
      .def_rw("segments", &plan_candidate_t::segments)
      .def_rw("grasp_events", &plan_candidate_t::grasp_events)
      .def_rw("score", &plan_candidate_t::score)
      .def_rw("diagnostics", &plan_candidate_t::diagnostics);

  nb::class_<plan_result_t>(module, "PlanResult")
      .def(nb::init<>())
      .def_rw("status", &plan_result_t::status)
      .def_rw("candidates", &plan_result_t::candidates)
      .def_rw("selected_index", &plan_result_t::selected_index)
      .def_rw("failure", &plan_result_t::failure);
}

} // namespace stacking_core::python
