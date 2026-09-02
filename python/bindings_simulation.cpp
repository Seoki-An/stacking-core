#include "bindings_common.hpp"

#include <stacking_core/posegen.hpp>
#include <stacking_core/simulation.hpp>

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace nb = nanobind;

namespace {

using namespace stacking_core;
using namespace stacking_core::python;

Matrix3X dsf_nodes(array_t const &value) {
  if (value.ndim() != 2 || value.shape(1) != 3 || value.shape(0) == 0) {
    throw std::invalid_argument("DSF nodes must have shape (N, 3)");
  }
  Scalar const *data = value.data();
  Matrix3X nodes(3, value.shape(0));
  for (Eigen::Index index = 0;
       index < static_cast<Eigen::Index>(value.shape(0)); ++index) {
    nodes.col(index) =
        Vector3{data[index * 3], data[index * 3 + 1], data[index * 3 + 2]};
  }
  return nodes;
}

struct python_simulation_result_t {
  python_scene_snapshot_t snapshot;
  std::vector<contact_t> contacts;
  simulation_solver_stats_t solver;
};

struct python_posegen_problem_t {
  posegen_problem_t value;
};

python_simulation_result_t python_result(simulation_result_t result) {
  return python_simulation_result_t{
      .snapshot = python_scene_snapshot_t{.value = std::move(result.snapshot)},
      .contacts = std::move(result.contacts),
      .solver = result.solver,
  };
}

nb::dict contact_dict(contact_t const &contact) {
  nb::dict value;
  value["entity_first"] = contact.feature.pair.first.entity.value();
  value["geometry_first"] = contact.feature.pair.first.geometry.value();
  value["entity_second"] = contact.feature.pair.second.entity.value();
  value["geometry_second"] = contact.feature.pair.second.geometry.value();
  value["gap"] = contact.feature.gap;
  value["point_first"] = contact.feature.point_first;
  value["point_second"] = contact.feature.point_second;
  value["normal"] = contact.feature.normal;
  value["friction"] = contact.friction;
  return value;
}

template <typename Value>
nb::dict entity_map(std::map<EntityId, Value> const &values) {
  nb::dict result;
  for (auto const &[entity, value] : values) {
    result[nb::int_(entity.value())] = nb::cast(value);
  }
  return result;
}

} // namespace

void stacking_core::python::bind_simulation(nb::module_ &module) {
  using namespace stacking_core;

  nb::object const zero_motion = nb::module_::import_("numpy").attr("zeros")(3);

  nb::enum_<mobility_e>(module, "Mobility")
      .value("STATIC", mobility_e::static_body)
      .value("KINEMATIC", mobility_e::kinematic)
      .value("DYNAMIC", mobility_e::dynamic);

  nb::enum_<simulation_contact_model_e>(module, "ContactModel")
      .value("SINGLE_POINT", simulation_contact_model_e::single_point)
      .value("LIMIT_SURFACE_4D", simulation_contact_model_e::limit_surface_4d);

  nb::class_<limit_surface_projection_config_t>(module,
                                                "LimitSurfaceProjectionConfig")
      .def(nb::init<>())
      .def_rw("max_iters", &limit_surface_projection_config_t::max_iters)
      .def_rw("tol", &limit_surface_projection_config_t::tol)
      .def_rw("friction_ratio_thresh",
              &limit_surface_projection_config_t::friction_ratio_thresh);

  nb::class_<simulation_solver_config_t>(module, "SolverConfig")
      .def(nb::init<>())
      .def_rw("damping", &simulation_solver_config_t::damping)
      .def_rw("beta_init", &simulation_solver_config_t::beta_init)
      .def_rw("beta_update_interval",
              &simulation_solver_config_t::beta_update_interval)
      .def_rw("beta_min", &simulation_solver_config_t::beta_min)
      .def_rw("beta_max", &simulation_solver_config_t::beta_max)
      .def_rw("max_iters", &simulation_solver_config_t::max_iters)
      .def_rw("tol_abs", &simulation_solver_config_t::tol_abs)
      .def_rw("tol_rel", &simulation_solver_config_t::tol_rel)
      .def_rw("stagnation_window",
              &simulation_solver_config_t::stagnation_window)
      .def_rw("stagnation_tol", &simulation_solver_config_t::stagnation_tol)
      .def_rw("stagnation_min_metric",
              &simulation_solver_config_t::stagnation_min_metric)
      .def_rw("stagnation_headroom",
              &simulation_solver_config_t::stagnation_headroom)
      .def_rw("stagnation_max_metric",
              &simulation_solver_config_t::stagnation_max_metric);

  nb::class_<simulation_contact_config_t>(module, "ContactConfig")
      .def(nb::init<>())
      .def_rw("model", &simulation_contact_config_t::model)
      .def_rw("error_reduction_ratio",
              &simulation_contact_config_t::error_reduction_ratio)
      .def_prop_rw(
          "body_error_reduction_ratio",
          [](simulation_contact_config_t const &config) {
            return entity_map(config.body_error_reduction_ratio);
          },
          [](simulation_contact_config_t &config, nb::dict const &values) {
            std::map<EntityId, Scalar> ratios;
            for (auto const &[key, value] : values) {
              ratios.emplace(EntityId{nb::cast<std::uint64_t>(key)},
                             nb::cast<Scalar>(value));
            }
            config.body_error_reduction_ratio = std::move(ratios);
          })
      .def_rw("detection_margin",
              &simulation_contact_config_t::detection_margin)
      .def_rw("patch_eps", &simulation_contact_config_t::patch_eps)
      .def_rw("patch_damping", &simulation_contact_config_t::patch_damping)
      .def_rw("projection", &simulation_contact_config_t::projection);

  nb::class_<simulation_config_t>(module, "SimulationConfig")
      .def(nb::init<>())
      .def_prop_rw(
          "gravity",
          [](simulation_config_t const &config) { return config.gravity; },
          [](simulation_config_t &config, array_t const &value) {
            config.gravity = vector3_from_array(value, "gravity");
          })
      .def_rw("contact", &simulation_config_t::contact)
      .def_rw("solver", &simulation_config_t::solver);

  nb::class_<BodyModel>(module, "BodyModel")
      .def_static(
          "dsf",
          [](std::uint64_t id, array_t const &nodes, Scalar mass,
             array_t const &inertia, Scalar friction, int sharpness,
             Scalar epsilon, std::uint64_t geometry_id) {
            std::vector<geometry_config_t> geometries;
            geometries.emplace_back(dsf_vert_geometry_config_t{
                .properties =
                    geometry_properties_t{
                        .id = GeometryId{geometry_id},
                        .body_from_geometry = pose_t{},
                        .material = material_t{.friction = friction},
                    },
                .nodes = dsf_nodes(nodes),
                .sharpness = sharpness,
                .epsilon = epsilon,
            });
            return std::make_shared<BodyModel>(body_model_config_t{
                .id = BodyModelId{id},
                .inertial =
                    inertial_t{
                        .body_from_inertial = pose_t{},
                        .mass = mass,
                        .inertia = matrix3_from_array(inertia, "inertia"),
                    },
                .geometries = std::move(geometries),
            });
          },
          nb::arg("id"), nb::arg("nodes"), nb::arg("mass"), nb::arg("inertia"),
          nb::arg("friction") = 1.0, nb::arg("sharpness") = 20,
          nb::arg("epsilon") = std::numeric_limits<Scalar>::epsilon(),
          nb::arg("geometry_id") = 1)
      .def_static(
          "dsf_components",
          [](std::uint64_t id, nb::sequence const &component_nodes, Scalar mass,
             array_t const &inertia, std::vector<Scalar> const &frictions,
             std::vector<int> const &sharpnesses, Scalar epsilon) {
            nb::ssize_t const count = nb::len(component_nodes);
            if (count == 0) {
              throw std::invalid_argument(
                  "DSF component nodes must not be empty");
            }
            if (frictions.size() != static_cast<std::size_t>(count) ||
                sharpnesses.size() != static_cast<std::size_t>(count)) {
              throw std::invalid_argument(
                  "DSF component properties must match the component count");
            }

            std::vector<geometry_config_t> geometries;
            geometries.reserve(static_cast<std::size_t>(count));
            for (nb::ssize_t index = 0; index < count; ++index) {
              array_t const nodes = nb::cast<array_t>(component_nodes[index]);
              geometries.emplace_back(dsf_vert_geometry_config_t{
                  .properties =
                      geometry_properties_t{
                          .id =
                              GeometryId{static_cast<std::uint64_t>(index + 1)},
                          .body_from_geometry = pose_t{},
                          .material =
                              material_t{.friction =
                                             frictions[static_cast<std::size_t>(
                                                 index)]},
                      },
                  .nodes = dsf_nodes(nodes),
                  .sharpness = sharpnesses[static_cast<std::size_t>(index)],
                  .epsilon = epsilon,
              });
            }
            return std::make_shared<BodyModel>(body_model_config_t{
                .id = BodyModelId{id},
                .inertial =
                    inertial_t{
                        .body_from_inertial = pose_t{},
                        .mass = mass,
                        .inertia = matrix3_from_array(inertia, "inertia"),
                    },
                .geometries = std::move(geometries),
            });
          },
          nb::arg("id"), nb::arg("component_nodes"), nb::arg("mass"),
          nb::arg("inertia"), nb::arg("frictions"), nb::arg("sharpnesses"),
          nb::arg("epsilon") = std::numeric_limits<Scalar>::epsilon())
      .def_static(
          "plane",
          [](std::uint64_t id, Scalar friction, std::uint64_t geometry_id) {
            std::vector<geometry_config_t> geometries;
            geometries.emplace_back(plane_geometry_config_t{
                .properties =
                    geometry_properties_t{
                        .id = GeometryId{geometry_id},
                        .body_from_geometry = pose_t{},
                        .material = material_t{.friction = friction},
                    },
            });
            return std::make_shared<BodyModel>(body_model_config_t{
                .id = BodyModelId{id},
                .inertial = std::nullopt,
                .geometries = std::move(geometries),
            });
          },
          nb::arg("id"), nb::arg("friction") = 1.0, nb::arg("geometry_id") = 1)
      .def_prop_ro("id",
                   [](BodyModel const &model) { return model.id().value(); })
      .def_prop_ro("geometry_count", &BodyModel::geometryCount)
      .def_prop_ro("has_inertial", &BodyModel::hasInertial)
      .def_prop_ro("mass",
                   [](BodyModel const &model) -> nb::object {
                     if (!model.hasInertial()) {
                       return nb::none();
                     }
                     return nb::float_(model.inertial().mass);
                   })
      .def_prop_ro("inertia", [](BodyModel const &model) -> nb::object {
        if (!model.hasInertial()) {
          return nb::none();
        }
        return nb::cast(model.inertial().inertia);
      });

  nb::class_<BodyInstance>(module, "BodyInstance")
      .def(nb::new_([](std::uint64_t id, nb::typed<nb::handle, BodyModel> model,
                       array_t const &pose, array_t const &linear,
                       array_t const &angular, mobility_e mobility) {
             pose_t const frame_from_body = pose_from_array(pose);
             std::shared_ptr<BodyModel> model_ptr =
                 nb::cast<std::shared_ptr<BodyModel>>(model);
             return BodyInstance{body_instance_config_t{
                 .id = EntityId{id},
                 .model = std::move(model_ptr),
                 .frame_from_body = frame_from_body,
                 .motion =
                     motion_t{
                         .linear = vector3_from_array(linear, "linear"),
                         .angular = vector3_from_array(angular, "angular"),
                     },
                 .mobility = mobility,
             }};
           }),
           nb::arg("id"), nb::arg("model"), nb::arg("pose"),
           nb::arg("linear") = zero_motion, nb::arg("angular") = zero_motion,
           nb::arg("mobility") = mobility_e::dynamic)
      .def_prop_ro("id",
                   [](BodyInstance const &body) { return body.id().value(); })
      .def_prop_ro(
          "model_id",
          [](BodyInstance const &body) { return body.model().id().value(); })
      .def_prop_ro("pose",
                   [](BodyInstance const &body) {
                     return pose_vector(body.frameFromBody());
                   })
      .def_prop_ro(
          "linear",
          [](BodyInstance const &body) { return body.motion().linear; })
      .def_prop_ro(
          "angular",
          [](BodyInstance const &body) { return body.motion().angular; })
      .def_prop_ro("mobility", &BodyInstance::mobility);

  nb::class_<python_scene_snapshot_t>(module, "SceneSnapshot")
      .def(nb::new_([](std::uint64_t frame_id,
                       std::vector<BodyInstance> bodies) {
             return python_scene_snapshot_t{
                 .value =
                     std::make_shared<SceneSnapshot>(scene_snapshot_config_t{
                         .frame = FrameId{frame_id},
                         .bodies = std::move(bodies),
                     }),
             };
           }),
           nb::arg("frame_id"), nb::arg("bodies"))
      .def_prop_ro("frame_id",
                   [](python_scene_snapshot_t const &scene) {
                     return scene.value->frame().value();
                   })
      .def_prop_ro("body_count",
                   [](python_scene_snapshot_t const &scene) {
                     return scene.value->bodyCount();
                   })
      .def("body_pose",
           [](python_scene_snapshot_t const &scene, std::uint64_t id) {
             return pose_vector(
                 scene.value->body(EntityId{id}).frameFromBody());
           })
      .def("body_linear_velocity",
           [](python_scene_snapshot_t const &scene, std::uint64_t id) {
             return scene.value->body(EntityId{id}).motion().linear;
           })
      .def("body_angular_velocity",
           [](python_scene_snapshot_t const &scene, std::uint64_t id) {
             return scene.value->body(EntityId{id}).motion().angular;
           });

  nb::class_<posegen_trust_region_config_t>(module, "PosegenTrustRegionConfig")
      .def(nb::init<>())
      .def_rw("max_iters", &posegen_trust_region_config_t::max_iters)
      .def_rw("eps", &posegen_trust_region_config_t::eps)
      .def_rw("delta_init", &posegen_trust_region_config_t::delta_init)
      .def_rw("delta_max", &posegen_trust_region_config_t::delta_max)
      .def_rw("delta_reduction_rate",
              &posegen_trust_region_config_t::delta_reduction_rate)
      .def_rw("delta_expansion_rate",
              &posegen_trust_region_config_t::delta_expansion_rate)
      .def_rw("delta_lower_thresh",
              &posegen_trust_region_config_t::delta_lower_thresh)
      .def_rw("delta_upper_thresh",
              &posegen_trust_region_config_t::delta_upper_thresh)
      .def_rw("improvement_thresh",
              &posegen_trust_region_config_t::improvement_thresh)
      .def_rw("tol", &posegen_trust_region_config_t::tol);

  nb::class_<posegen_hausdorff_config_t>(module, "PosegenHausdorffConfig")
      .def(nb::init<>())
      .def_rw("eps", &posegen_hausdorff_config_t::eps)
      .def_rw("max_iters", &posegen_hausdorff_config_t::max_iters)
      .def_rw("error_tol", &posegen_hausdorff_config_t::error_tol)
      .def_rw("radius_init", &posegen_hausdorff_config_t::radius_init)
      .def_rw("radius_reduction_rate",
              &posegen_hausdorff_config_t::radius_reduction_rate)
      .def_rw("radius_expansion_rate",
              &posegen_hausdorff_config_t::radius_expansion_rate)
      .def_rw("gain_ratio_lower_thresh",
              &posegen_hausdorff_config_t::gain_ratio_lower_thresh)
      .def_rw("gain_ratio_upper_thresh",
              &posegen_hausdorff_config_t::gain_ratio_upper_thresh)
      .def_rw("gain_ratio_max", &posegen_hausdorff_config_t::gain_ratio_max);

  nb::class_<posegen_objective_config_t>(module, "PosegenObjectiveConfig")
      .def(nb::init<>())
      .def_rw("rho", &posegen_objective_config_t::rho)
      .def_rw("narrow_phase_scene_tol",
              &posegen_objective_config_t::narrow_phase_scene_tol)
      .def_rw("narrow_phase_candidate_tol",
              &posegen_objective_config_t::narrow_phase_candidate_tol)
      .def_rw("eps_gap", &posegen_objective_config_t::eps_gap)
      .def_rw("eps_comp", &posegen_objective_config_t::eps_comp)
      .def_rw("eps_cone", &posegen_objective_config_t::eps_cone)
      .def_rw("eps_target", &posegen_objective_config_t::eps_target)
      .def_rw("k_comp", &posegen_objective_config_t::k_comp)
      .def_prop_rw(
          "k_wrench",
          [](posegen_objective_config_t const &config) {
            return config.k_wrench;
          },
          [](posegen_objective_config_t &config, array_t const &value) {
            config.k_wrench = matrix6_from_array(value, "k_wrench");
          })
      .def_rw("k_gap", &posegen_objective_config_t::k_gap)
      .def_rw("k_gap_c", &posegen_objective_config_t::k_gap_c)
      .def_rw("k_target", &posegen_objective_config_t::k_target)
      .def_rw("k_potential", &posegen_objective_config_t::k_potential)
      .def_rw("k_xy", &posegen_objective_config_t::k_xy)
      .def_rw("k_box", &posegen_objective_config_t::k_box)
      .def_rw("k_reg", &posegen_objective_config_t::k_reg)
      .def_rw("k_lower", &posegen_objective_config_t::k_lower)
      .def_rw("w_box", &posegen_objective_config_t::w_box)
      .def_rw("gravity", &posegen_objective_config_t::gravity)
      .def_rw("ground_height", &posegen_objective_config_t::ground_height);

  nb::enum_<posegen_force_solver_e>(module, "PosegenForceSolver")
      .value("GRAPH_ADMM", posegen_force_solver_e::graph_admm)
      .value("INTERIOR_POINT", posegen_force_solver_e::interior_point);

  nb::class_<posegen_force_solver_config_t>(module, "PosegenForceSolverConfig")
      .def(nb::init<>())
      .def_rw("method", &posegen_force_solver_config_t::method)
      .def_rw("max_iters", &posegen_force_solver_config_t::max_iters)
      .def_rw("beta_consensus", &posegen_force_solver_config_t::beta_consensus)
      .def_rw("beta_contact", &posegen_force_solver_config_t::beta_contact)
      .def_rw("beta_update_interval",
              &posegen_force_solver_config_t::beta_update_interval)
      .def_rw("tol_abs", &posegen_force_solver_config_t::tol_abs)
      .def_rw("tol_rel", &posegen_force_solver_config_t::tol_rel);

  nb::class_<posegen_config_t>(module, "PosegenConfig")
      .def(nb::init<>())
      .def_rw("trust_region", &posegen_config_t::trust_region)
      .def_rw("hausdorff", &posegen_config_t::hausdorff)
      .def_rw("objective", &posegen_config_t::objective)
      .def_rw("force_solver", &posegen_config_t::force_solver);

  nb::class_<python_posegen_problem_t>(module, "PosegenProblem")
      .def(nb::new_([](python_scene_snapshot_t const &scene,
                       std::uint64_t candidate,
                       std::vector<std::uint64_t> const &targets,
                       std::vector<std::uint64_t> const &boundaries) {
             std::vector<EntityId> ids;
             ids.reserve(scene.value->bodyCount());
             for (std::size_t index = 0; index < scene.value->bodyCount();
                  ++index) {
               ids.push_back(scene.value->body(index).id());
             }
             auto convert = [](std::vector<std::uint64_t> const &values) {
               std::vector<EntityId> result;
               result.reserve(values.size());
               for (std::uint64_t value : values) {
                 result.emplace_back(value);
               }
               return result;
             };
             return python_posegen_problem_t{
                 .value = posegen_problem_t{
                     .scene = SceneView{scene.value, std::move(ids)},
                     .candidate = EntityId{candidate},
                     .targets = convert(targets),
                     .boundaries = convert(boundaries),
                 }};
           }),
           nb::arg("scene"), nb::arg("candidate"),
           nb::arg("targets") = std::vector<std::uint64_t>{},
           nb::arg("boundaries") = std::vector<std::uint64_t>{});

  nb::class_<posegen_force_solver_stats_t>(module, "PosegenForceSolverStats")
      .def_ro("iters", &posegen_force_solver_stats_t::iters)
      .def_ro("converged", &posegen_force_solver_stats_t::converged)
      .def_ro("primal_residual", &posegen_force_solver_stats_t::primal_residual)
      .def_ro("dual_residual", &posegen_force_solver_stats_t::dual_residual);

  nb::class_<posegen_solver_stats_t>(module, "PosegenSolverStats")
      .def_ro("iters", &posegen_solver_stats_t::iters)
      .def_ro("accepted_iters", &posegen_solver_stats_t::accepted_iters)
      .def_ro("objective_evals", &posegen_solver_stats_t::objective_evals)
      .def_ro("converged", &posegen_solver_stats_t::converged)
      .def_ro("grad_norm", &posegen_solver_stats_t::grad_norm)
      .def_ro("trust_region_radius",
              &posegen_solver_stats_t::trust_region_radius)
      .def_ro("scene_graph_rebuilds",
              &posegen_solver_stats_t::scene_graph_rebuilds)
      .def_ro("scene_graph_reuses", &posegen_solver_stats_t::scene_graph_reuses)
      .def_ro("force_solver", &posegen_solver_stats_t::force_solver);

  nb::class_<posegen_result_t>(module, "PosegenResult")
      .def_prop_ro("optimal_pose",
                   [](posegen_result_t const &result) {
                     return pose_vector(result.optimal_pose);
                   })
      .def_ro("candidate_contact_forces",
              &posegen_result_t::candidate_contact_forces)
      .def_ro("c_feq", &posegen_result_t::c_feq)
      .def_ro("c_comp", &posegen_result_t::c_comp)
      .def_ro("c_gap", &posegen_result_t::c_gap)
      .def_prop_ro("net_wrench",
                   [](posegen_result_t const &result) {
                     return entity_map(result.net_wrench);
                   })
      .def_prop_ro("contact_force",
                   [](posegen_result_t const &result) {
                     return entity_map(result.contact_force);
                   })
      .def_prop_ro("contact_point",
                   [](posegen_result_t const &result) {
                     return entity_map(result.contact_point);
                   })
      .def_prop_ro("contact_normal",
                   [](posegen_result_t const &result) {
                     return entity_map(result.contact_normal);
                   })
      .def_ro("solver", &posegen_result_t::solver);

  nb::class_<PoseGenerator>(module, "PoseGenerator")
      .def(nb::init<posegen_config_t>(), nb::arg("config") = posegen_config_t{})
      .def_prop_rw(
          "config",
          [](PoseGenerator const &generator) { return generator.config(); },
          &PoseGenerator::setConfig)
      .def(
          "solve",
          [](PoseGenerator &generator,
             python_posegen_problem_t const &problem) {
            nb::gil_scoped_release release;
            return generator.solve(problem.value);
          },
          nb::arg("problem"));

  nb::class_<simulation_solver_stats_t>(module, "SolverStats")
      .def_ro("iters", &simulation_solver_stats_t::iters)
      .def_ro("converged", &simulation_solver_stats_t::converged)
      .def_ro("primal_residual", &simulation_solver_stats_t::primal_residual)
      .def_ro("dual_residual", &simulation_solver_stats_t::dual_residual);

  nb::class_<python_simulation_result_t>(module, "SimulationResult")
      .def_ro("snapshot", &python_simulation_result_t::snapshot)
      .def_prop_ro("contacts",
                   [](python_simulation_result_t const &result) {
                     nb::list values;
                     for (contact_t const &contact : result.contacts) {
                       values.append(contact_dict(contact));
                     }
                     return values;
                   })
      .def_ro("solver", &python_simulation_result_t::solver);

  nb::class_<Simulator>(module, "Simulator")
      .def(nb::init<simulation_config_t>(),
           nb::arg("config") = simulation_config_t{})
      .def(
          "step",
          [](Simulator &simulator, python_scene_snapshot_t const &scene,
             Scalar dt) {
            nb::gil_scoped_release release;
            return python_result(simulator.step(*scene.value, dt));
          },
          nb::arg("scene"), nb::arg("dt"))
      .def(
          "step_n",
          [](Simulator &simulator, python_scene_snapshot_t const &scene,
             Scalar dt, std::size_t count) {
            nb::gil_scoped_release release;
            return python_result(simulator.step_n(*scene.value, dt, count));
          },
          nb::arg("scene"), nb::arg("dt"), nb::arg("count"))
      .def("clear_warm_start", &Simulator::clearWarmStart);
}
