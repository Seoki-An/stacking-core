#include "bindings_common.hpp"

#include <stacking_core/planner/grasp.hpp>
#include <stacking_core/planner/grasp/sampling.hpp>
#include <stacking_core/planner/grasp/simulation.hpp>
#include <stacking_core/planner/stable_pose.hpp>

#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace nb = nanobind;

namespace stacking_core::python {

void bind_planner_grasp(nb::module_ &module) {
  nb::enum_<grasp_contact_side_e>(module, "GraspContactSide")
      .value("LEFT", grasp_contact_side_e::left)
      .value("RIGHT", grasp_contact_side_e::right);

  nb::enum_<grasp_contact_type_e>(module, "GraspContactType")
      .value("PAD", grasp_contact_type_e::pad)
      .value("TOOTH", grasp_contact_type_e::tooth);

  nb::enum_<grasp_sampling_strategy_e>(module, "GraspSamplingStrategy")
      .value("ANTIPODAL_SURFACE", grasp_sampling_strategy_e::antipodal_surface)
      .value("PARALLEL_JAW", grasp_sampling_strategy_e::parallel_jaw);

  nb::class_<grasp_link_body_t>(module, "GraspLinkBody")
      .def(nb::new_([](std::uint64_t link, std::uint64_t entity,
                       std::shared_ptr<BodyModel> body_model) {
             return grasp_link_body_t{
                 .link = LinkId{link},
                 .entity = EntityId{entity},
                 .body_model = std::move(body_model),
             };
           }),
           nb::arg("link"), nb::arg("entity"), nb::arg("body_model"))
      .def_prop_ro(
          "link",
          [](grasp_link_body_t const &body) { return body.link.value(); })
      .def_prop_ro(
          "entity",
          [](grasp_link_body_t const &body) { return body.entity.value(); })
      .def_ro("body_model", &grasp_link_body_t::body_model);

  nb::class_<grasp_contact_geometry_t>(module, "GraspContactGeometry")
      .def(
          nb::new_([](std::uint64_t entity, std::uint64_t geometry,
                      grasp_contact_side_e side, grasp_contact_type_e type,
                      int order, bool enforce_contact, bool contributes_force) {
            return grasp_contact_geometry_t{
                .entity = EntityId{entity},
                .geometry = GeometryId{geometry},
                .side = side,
                .type = type,
                .order = order,
                .enforce_contact = enforce_contact,
                .contributes_force = contributes_force,
            };
          }),
          nb::arg("entity"), nb::arg("geometry"), nb::arg("side"),
          nb::arg("type") = grasp_contact_type_e::pad, nb::arg("order") = 0,
          nb::arg("enforce_contact") = true,
          nb::arg("contributes_force") = true)
      .def_prop_ro("entity",
                   [](grasp_contact_geometry_t const &value) {
                     return value.entity.value();
                   })
      .def_prop_ro("geometry",
                   [](grasp_contact_geometry_t const &value) {
                     return value.geometry.value();
                   })
      .def_rw("side", &grasp_contact_geometry_t::side)
      .def_rw("type", &grasp_contact_geometry_t::type)
      .def_rw("order", &grasp_contact_geometry_t::order)
      .def_rw("enforce_contact", &grasp_contact_geometry_t::enforce_contact)
      .def_rw("contributes_force",
              &grasp_contact_geometry_t::contributes_force);

  nb::class_<gripper_model_t>(module, "GripperModel")
      .def(
          nb::new_(
              [](KinematicState state, std::uint64_t root_link,
                 array_t const &grasp_from_root, Eigen::VectorXd opening_offset,
                 Eigen::VectorXd opening_direction, Scalar opening_lower,
                 Scalar opening_upper,
                 std::vector<grasp_link_body_t> collision_bodies,
                 std::vector<grasp_contact_geometry_t> contact_geometries) {
                return gripper_model_t{
                    .state = std::move(state),
                    .root_link = LinkId{root_link},
                    .grasp_from_root =
                        pose_from_array(grasp_from_root, "grasp_from_root"),
                    .opening_offset = std::move(opening_offset),
                    .opening_direction = std::move(opening_direction),
                    .opening_lower = opening_lower,
                    .opening_upper = opening_upper,
                    .collision_bodies = std::move(collision_bodies),
                    .contact_geometries = std::move(contact_geometries),
                };
              }),
          nb::arg("state"), nb::arg("root_link"), nb::arg("grasp_from_root"),
          nb::arg("opening_offset"), nb::arg("opening_direction"),
          nb::arg("opening_lower") = 0.0, nb::arg("opening_upper") = 1.0,
          nb::arg("collision_bodies") = std::vector<grasp_link_body_t>{},
          nb::arg("contact_geometries") =
              std::vector<grasp_contact_geometry_t>{})
      .def_rw("state", &gripper_model_t::state)
      .def_prop_ro("root_link",
                   [](gripper_model_t const &gripper) {
                     return gripper.root_link.value();
                   })
      .def_prop_rw(
          "grasp_from_root",
          [](gripper_model_t const &gripper) {
            return pose_vector(gripper.grasp_from_root);
          },
          [](gripper_model_t &gripper, array_t const &value) {
            gripper.grasp_from_root = pose_from_array(value, "grasp_from_root");
          })
      .def_rw("opening_offset", &gripper_model_t::opening_offset)
      .def_rw("opening_direction", &gripper_model_t::opening_direction)
      .def_rw("opening_lower", &gripper_model_t::opening_lower)
      .def_rw("opening_upper", &gripper_model_t::opening_upper)
      .def_rw("collision_bodies", &gripper_model_t::collision_bodies)
      .def_rw("contact_geometries", &gripper_model_t::contact_geometries);

  nb::class_<grasp_cost_weights_t>(module, "GraspCostWeights")
      .def(nb::init<>())
      .def_rw("antipodal_normal", &grasp_cost_weights_t::antipodal_normal)
      .def_rw("antipodal_position", &grasp_cost_weights_t::antipodal_position)
      .def_rw("align", &grasp_cost_weights_t::align)
      .def_rw("enclosure", &grasp_cost_weights_t::enclosure)
      .def_rw("radial_distance", &grasp_cost_weights_t::radial_distance)
      .def_rw("center_distance", &grasp_cost_weights_t::center_distance)
      .def_rw("contact", &grasp_cost_weights_t::contact)
      .def_rw("teeth_fit", &grasp_cost_weights_t::teeth_fit)
      .def_rw("teeth_align", &grasp_cost_weights_t::teeth_align)
      .def_rw("flatness", &grasp_cost_weights_t::flatness);

  nb::class_<grasp_score_weights_t>(module, "GraspScoreWeights")
      .def(nb::init<>())
      .def_rw("center_distance", &grasp_score_weights_t::center_distance)
      .def_rw("align", &grasp_score_weights_t::align)
      .def_rw("enclosure", &grasp_score_weights_t::enclosure)
      .def_rw("radial_distance", &grasp_score_weights_t::radial_distance)
      .def_rw("teeth_gap", &grasp_score_weights_t::teeth_gap)
      .def_rw("teeth_fit", &grasp_score_weights_t::teeth_fit)
      .def_rw("teeth_align", &grasp_score_weights_t::teeth_align);

  nb::class_<grasp_trust_region_config_t>(module, "GraspTrustRegionConfig")
      .def(nb::init<>())
      .def_rw("max_iters", &grasp_trust_region_config_t::max_iters)
      .def_rw("subproblem_tol", &grasp_trust_region_config_t::subproblem_tol)
      .def_rw("radius_max", &grasp_trust_region_config_t::radius_max)
      .def_rw("radius_init", &grasp_trust_region_config_t::radius_init)
      .def_rw("radius_reduction",
              &grasp_trust_region_config_t::radius_reduction)
      .def_rw("radius_expansion",
              &grasp_trust_region_config_t::radius_expansion)
      .def_rw("gain_ratio_lower",
              &grasp_trust_region_config_t::gain_ratio_lower)
      .def_rw("gain_ratio_upper",
              &grasp_trust_region_config_t::gain_ratio_upper)
      .def_rw("improvement_tol", &grasp_trust_region_config_t::improvement_tol)
      .def_rw("step_tol", &grasp_trust_region_config_t::step_tol);

  nb::class_<grasp_alm_config_t>(module, "GraspAlmConfig")
      .def(nb::init<>())
      .def_rw("max_iters", &grasp_alm_config_t::max_iters)
      .def_rw("inequality_tol", &grasp_alm_config_t::inequality_tol)
      .def_rw("equality_tol", &grasp_alm_config_t::equality_tol)
      .def_rw("inequality_penalty_init",
              &grasp_alm_config_t::inequality_penalty_init)
      .def_rw("equality_penalty_init",
              &grasp_alm_config_t::equality_penalty_init)
      .def_rw("inequality_penalty_increase",
              &grasp_alm_config_t::inequality_penalty_increase)
      .def_rw("equality_penalty_increase",
              &grasp_alm_config_t::equality_penalty_increase);

  nb::class_<grasp_force_weights_t>(module, "GraspForceWeights")
      .def(nb::init<>())
      .def_rw("wrench", &grasp_force_weights_t::wrench)
      .def_rw("complementarity", &grasp_force_weights_t::complementarity)
      .def_rw("cone", &grasp_force_weights_t::cone)
      .def_rw("moment", &grasp_force_weights_t::moment);

  nb::class_<grasp_force_config_t>(module, "GraspForceConfig")
      .def(nb::init<>())
      .def_rw("enabled", &grasp_force_config_t::enabled)
      .def_rw("weight", &grasp_force_config_t::weight)
      .def_rw("friction", &grasp_force_config_t::friction)
      .def_rw("wrench_scale", &grasp_force_config_t::wrench_scale)
      .def_rw("damping", &grasp_force_config_t::damping);

  nb::class_<grasp_generation_config_t>(module, "GraspGenerationConfig")
      .def(nb::init<>())
      .def_rw("cost", &grasp_generation_config_t::cost)
      .def_rw("score", &grasp_generation_config_t::score)
      .def_rw("trust_region", &grasp_generation_config_t::trust_region)
      .def_rw("alm", &grasp_generation_config_t::alm)
      .def_rw("force", &grasp_generation_config_t::force)
      .def_rw("separate_margin", &grasp_generation_config_t::separate_margin)
      .def_rw("plane_separate_margin",
              &grasp_generation_config_t::plane_separate_margin)
      .def_rw("contact_margin", &grasp_generation_config_t::contact_margin);

  nb::class_<grasp_sampling_config_t>(module, "GraspSamplingConfig")
      .def(nb::init<>())
      .def_rw("strategy", &grasp_sampling_config_t::strategy)
      .def_rw("max_seeds", &grasp_sampling_config_t::max_seeds)
      .def_rw("dir_samples", &grasp_sampling_config_t::dir_samples)
      .def_rw("spin_samples", &grasp_sampling_config_t::spin_samples)
      .def_rw("spin_step", &grasp_sampling_config_t::spin_step)
      .def_rw("include_flipped", &grasp_sampling_config_t::include_flipped)
      .def_rw("retreat_distance", &grasp_sampling_config_t::retreat_distance)
      .def_rw("max_target_width", &grasp_sampling_config_t::max_target_width)
      .def_rw("aperture_margin", &grasp_sampling_config_t::aperture_margin)
      .def_rw("scene_clearance_margin",
              &grasp_sampling_config_t::scene_clearance_margin)
      .def_rw("preferred_parallel_axis",
              &grasp_sampling_config_t::preferred_parallel_axis);

  nb::class_<grasp_simulation_config_t>(module, "GraspSimulationConfig")
      .def(nb::init<>())
      .def_rw("steps", &grasp_simulation_config_t::steps)
      .def_rw("dt", &grasp_simulation_config_t::dt)
      .def_rw("virtual_closing_effort",
              &grasp_simulation_config_t::virtual_closing_effort)
      .def_rw("virtual_approach_force",
              &grasp_simulation_config_t::virtual_approach_force)
      .def_rw("damping", &grasp_simulation_config_t::damping)
      .def_rw("friction", &grasp_simulation_config_t::friction)
      .def_rw("pgs_iters", &grasp_simulation_config_t::pgs_iters)
      .def_rw("error_reduction_ratio",
              &grasp_simulation_config_t::error_reduction_ratio)
      .def_rw("target_contact_margin",
              &grasp_simulation_config_t::target_contact_margin)
      .def_rw("obstacle_margin", &grasp_simulation_config_t::obstacle_margin)
      .def_rw("plane_obstacle_margin",
              &grasp_simulation_config_t::plane_obstacle_margin)
      .def_rw("active_impulse_tol",
              &grasp_simulation_config_t::active_impulse_tol)
      .def_rw("settled_velocity_tol",
              &grasp_simulation_config_t::settled_velocity_tol)
      .def_rw("relaxed_velocity_tol",
              &grasp_simulation_config_t::relaxed_velocity_tol)
      .def_rw("accept_relaxed", &grasp_simulation_config_t::accept_relaxed)
      .def_rw("grasp_offset_tol", &grasp_simulation_config_t::grasp_offset_tol);

  nb::class_<stable_pose_config_t>(module, "StablePoseConfig")
      .def(nb::init<>())
      .def_rw("sampling_level", &stable_pose_config_t::sampling_level)
      .def_rw("stable_eigenvalue_min",
              &stable_pose_config_t::stable_eigenvalue_min);

  nb::class_<grasp_candidate_t>(module, "GraspCandidate")
      .def(nb::init<>())
      .def_rw("grasp", &grasp_candidate_t::grasp)
      .def_rw("score", &grasp_candidate_t::score)
      .def_rw("contacts", &grasp_candidate_t::contacts)
      .def_rw("solver", &grasp_candidate_t::solver)
      .def_rw("failure", &grasp_candidate_t::failure);

  nb::class_<joint_grasp_candidate_t>(module, "JointGraspCandidate")
      .def(nb::init<>())
      .def_rw("grasp", &joint_grasp_candidate_t::grasp)
      .def_rw("positions", &joint_grasp_candidate_t::positions);
}

} // namespace stacking_core::python
