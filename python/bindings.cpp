#include <stacking_core/simulation.hpp>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace {

using namespace stacking_core;

using pose_vector_t = Eigen::Matrix<Scalar, 7, 1>;

using array_t = py::array_t<
  Scalar, py::array::c_style | py::array::forcecast>;

Vector3 vector3_from_array(array_t const& value, char const* name) {
  py::buffer_info const info = value.request();
  if (info.ndim != 1 || info.shape[0] != 3) {
    throw std::invalid_argument(std::string {name} + " must have shape (3,)");
  }
  auto const* data = static_cast<Scalar const*>(info.ptr);
  return Vector3 {data[0], data[1], data[2]};
}

Matrix3 matrix3_from_array(array_t const& value, char const* name) {
  py::buffer_info const info = value.request();
  if (info.ndim != 2 || info.shape[0] != 3 || info.shape[1] != 3) {
    throw std::invalid_argument(std::string {name} + " must have shape (3, 3)");
  }
  auto const* data = static_cast<Scalar const*>(info.ptr);
  Matrix3 result;
  for (Eigen::Index row = 0; row < 3; ++row) {
    for (Eigen::Index col = 0; col < 3; ++col) {
      result(row, col) = data[row * 3 + col];
    }
  }
  return result;
}

pose_t pose_from_array(array_t const& value) {
  py::buffer_info const info = value.request();
  if (info.ndim != 1 || info.shape[0] != 7) {
    throw std::invalid_argument(
      "pose must contain [x, y, z, qx, qy, qz, qw]");
  }
  auto const* data = static_cast<Scalar const*>(info.ptr);
  return pose_t {
    Vector3 {data[0], data[1], data[2]},
    Quaternion {data[6], data[3], data[4], data[5]},
  };
}

pose_vector_t pose_vector(pose_t const& pose) {
  pose_vector_t value;
  value << pose.position, pose.orientation.coeffs();
  return value;
}

Matrix3X dsf_nodes(array_t const& value) {
  py::buffer_info const info = value.request();
  if (info.ndim != 2 || info.shape[1] != 3 || info.shape[0] == 0) {
    throw std::invalid_argument("DSF nodes must have shape (N, 3)");
  }
  auto const* data = static_cast<Scalar const*>(info.ptr);
  Matrix3X nodes(3, info.shape[0]);
  for (Eigen::Index index = 0; index < info.shape[0]; ++index) {
    nodes.col(index) = Vector3 {
      data[index * 3], data[index * 3 + 1], data[index * 3 + 2]};
  }
  return nodes;
}

struct python_scene_snapshot_t {
  std::shared_ptr<SceneSnapshot const> value;
};

struct python_simulation_result_t {
  python_scene_snapshot_t snapshot;
  std::vector<contact_t> contacts;
  simulation_solver_stats_t solver;
};

python_simulation_result_t python_result(simulation_result_t result) {
  return python_simulation_result_t {
    .snapshot = python_scene_snapshot_t {.value = std::move(result.snapshot)},
    .contacts = std::move(result.contacts),
    .solver = result.solver,
  };
}

py::dict contact_dict(contact_t const& contact) {
  py::dict value;
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

}  // namespace

PYBIND11_MODULE(_native, module) {
  using namespace stacking_core;

  module.doc() = "Python bindings for stacking-core scene simulation";

  py::enum_<mobility_e>(module, "Mobility")
    .value("STATIC", mobility_e::static_body)
    .value("KINEMATIC", mobility_e::kinematic)
    .value("DYNAMIC", mobility_e::dynamic);

  py::enum_<simulation_contact_model_e>(module, "ContactModel")
    .value("SINGLE_POINT", simulation_contact_model_e::single_point)
    .value("LIMIT_SURFACE_4D", simulation_contact_model_e::limit_surface_4d);

  py::class_<limit_surface_projection_config_t>(
    module, "LimitSurfaceProjectionConfig")
    .def(py::init<>())
    .def_readwrite(
      "max_iters", &limit_surface_projection_config_t::max_iters)
    .def_readwrite("tol", &limit_surface_projection_config_t::tol)
    .def_readwrite(
      "friction_ratio_thresh",
      &limit_surface_projection_config_t::friction_ratio_thresh);

  py::class_<simulation_solver_config_t>(module, "SolverConfig")
    .def(py::init<>())
    .def_readwrite("damping", &simulation_solver_config_t::damping)
    .def_readwrite("beta_init", &simulation_solver_config_t::beta_init)
    .def_readwrite(
      "beta_update_interval",
      &simulation_solver_config_t::beta_update_interval)
    .def_readwrite("beta_min", &simulation_solver_config_t::beta_min)
    .def_readwrite("beta_max", &simulation_solver_config_t::beta_max)
    .def_readwrite("max_iters", &simulation_solver_config_t::max_iters)
    .def_readwrite("tol_abs", &simulation_solver_config_t::tol_abs)
    .def_readwrite("tol_rel", &simulation_solver_config_t::tol_rel)
    .def_readwrite(
      "stagnation_window", &simulation_solver_config_t::stagnation_window)
    .def_readwrite(
      "stagnation_tol", &simulation_solver_config_t::stagnation_tol)
    .def_readwrite(
      "stagnation_min_metric",
      &simulation_solver_config_t::stagnation_min_metric)
    .def_readwrite(
      "stagnation_headroom",
      &simulation_solver_config_t::stagnation_headroom)
    .def_readwrite(
      "stagnation_max_metric",
      &simulation_solver_config_t::stagnation_max_metric);

  py::class_<simulation_contact_config_t>(module, "ContactConfig")
    .def(py::init<>())
    .def_readwrite("model", &simulation_contact_config_t::model)
    .def_readwrite(
      "error_reduction_ratio",
      &simulation_contact_config_t::error_reduction_ratio)
    .def_readwrite(
      "detection_margin", &simulation_contact_config_t::detection_margin)
    .def_readwrite("patch_eps", &simulation_contact_config_t::patch_eps)
    .def_readwrite(
      "patch_damping", &simulation_contact_config_t::patch_damping)
    .def_readwrite(
      "projection", &simulation_contact_config_t::projection);

  py::class_<simulation_config_t>(module, "SimulationConfig")
    .def(py::init<>())
    .def_property(
      "gravity",
      [](simulation_config_t const& config) { return config.gravity; },
      [](simulation_config_t& config, array_t const& value) {
        config.gravity = vector3_from_array(value, "gravity");
      })
    .def_readwrite("contact", &simulation_config_t::contact)
    .def_readwrite("solver", &simulation_config_t::solver);

  py::class_<BodyModel, std::shared_ptr<BodyModel>>(module, "BodyModel")
    .def_static(
      "dsf",
      [](std::uint64_t id,
         array_t const& nodes,
         Scalar mass,
         array_t const& inertia,
         Scalar friction,
         int sharpness,
         Scalar epsilon,
         std::uint64_t geometry_id) {
        std::vector<geometry_config_t> geometries;
        geometries.emplace_back(dsf_vert_geometry_config_t {
          .properties = geometry_properties_t {
            .id = GeometryId {geometry_id},
            .body_from_geometry = pose_t {},
            .material = material_t {.friction = friction},
          },
          .nodes = dsf_nodes(nodes),
          .sharpness = sharpness,
          .epsilon = epsilon,
        });
        return std::make_shared<BodyModel>(body_model_config_t {
          .id = BodyModelId {id},
          .inertial = inertial_t {
            .body_from_inertial = pose_t {},
            .mass = mass,
            .inertia = matrix3_from_array(inertia, "inertia"),
          },
          .geometries = std::move(geometries),
        });
      },
      py::arg("id"),
      py::arg("nodes"),
      py::arg("mass"),
      py::arg("inertia"),
      py::arg("friction") = 1.0,
      py::arg("sharpness") = 20,
      py::arg("epsilon") = std::numeric_limits<Scalar>::epsilon(),
      py::arg("geometry_id") = 1)
    .def_static(
      "dsf_components",
      [](std::uint64_t id,
         py::sequence const& component_nodes,
         Scalar mass,
         array_t const& inertia,
         std::vector<Scalar> const& frictions,
         std::vector<int> const& sharpnesses,
         Scalar epsilon) {
        py::ssize_t const count = py::len(component_nodes);
        if (count == 0) {
          throw std::invalid_argument(
            "DSF component nodes must not be empty");
        }
        if (frictions.size() != static_cast<std::size_t>(count)
            || sharpnesses.size() != static_cast<std::size_t>(count)) {
          throw std::invalid_argument(
            "DSF component properties must match the component count");
        }

        std::vector<geometry_config_t> geometries;
        geometries.reserve(static_cast<std::size_t>(count));
        for (py::ssize_t index = 0; index < count; ++index) {
          array_t const nodes = py::cast<array_t>(component_nodes[index]);
          geometries.emplace_back(dsf_vert_geometry_config_t {
            .properties = geometry_properties_t {
              .id = GeometryId {static_cast<std::uint64_t>(index + 1)},
              .body_from_geometry = pose_t {},
              .material = material_t {
                .friction = frictions[static_cast<std::size_t>(index)]},
              },
            .nodes = dsf_nodes(nodes),
            .sharpness = sharpnesses[static_cast<std::size_t>(index)],
            .epsilon = epsilon,
          });
        }
        return std::make_shared<BodyModel>(body_model_config_t {
          .id = BodyModelId {id},
          .inertial = inertial_t {
            .body_from_inertial = pose_t {},
            .mass = mass,
            .inertia = matrix3_from_array(inertia, "inertia"),
          },
          .geometries = std::move(geometries),
        });
      },
      py::arg("id"),
      py::arg("component_nodes"),
      py::arg("mass"),
      py::arg("inertia"),
      py::arg("frictions"),
      py::arg("sharpnesses"),
      py::arg("epsilon") = std::numeric_limits<Scalar>::epsilon())
    .def_static(
      "plane",
      [](std::uint64_t id,
         Scalar friction,
         std::uint64_t geometry_id) {
        std::vector<geometry_config_t> geometries;
        geometries.emplace_back(plane_geometry_config_t {
          .properties = geometry_properties_t {
            .id = GeometryId {geometry_id},
            .body_from_geometry = pose_t {},
            .material = material_t {.friction = friction},
          },
        });
        return std::make_shared<BodyModel>(body_model_config_t {
          .id = BodyModelId {id},
          .inertial = std::nullopt,
          .geometries = std::move(geometries),
        });
      },
      py::arg("id"),
      py::arg("friction") = 1.0,
      py::arg("geometry_id") = 1)
    .def_property_readonly(
      "id", [](BodyModel const& model) { return model.id().value(); })
    .def_property_readonly(
      "geometry_count", &BodyModel::geometryCount)
    .def_property_readonly("has_inertial", &BodyModel::hasInertial)
    .def_property_readonly(
      "mass",
      [](BodyModel const& model) -> py::object {
        if (!model.hasInertial()) {
          return py::none();
        }
        return py::float_(model.inertial().mass);
      })
    .def_property_readonly(
      "inertia",
      [](BodyModel const& model) -> py::object {
        if (!model.hasInertial()) {
          return py::none();
        }
        return py::cast(model.inertial().inertia);
      });

  py::class_<BodyInstance>(module, "BodyInstance")
    .def(
      py::init([](std::uint64_t id,
                  std::shared_ptr<BodyModel> const& model,
                  array_t const& pose,
                  array_t const& linear,
                  array_t const& angular,
                  mobility_e mobility) {
        return BodyInstance {body_instance_config_t {
          .id = EntityId {id},
          .model = std::move(model),
          .frame_from_body = pose_from_array(pose),
          .motion = motion_t {
            .linear = vector3_from_array(linear, "linear"),
            .angular = vector3_from_array(angular, "angular"),
          },
          .mobility = mobility,
        }};
      }),
      py::arg("id"),
      py::arg("model"),
      py::arg("pose"),
      py::arg("linear") = std::array<Scalar, 3> {0.0, 0.0, 0.0},
      py::arg("angular") = std::array<Scalar, 3> {0.0, 0.0, 0.0},
      py::arg("mobility") = mobility_e::dynamic)
    .def_property_readonly(
      "id", [](BodyInstance const& body) { return body.id().value(); })
    .def_property_readonly(
      "model_id",
      [](BodyInstance const& body) { return body.model().id().value(); })
    .def_property_readonly(
      "pose",
      [](BodyInstance const& body) {
        return pose_vector(body.frameFromBody());
      })
    .def_property_readonly(
      "linear",
      [](BodyInstance const& body) { return body.motion().linear; })
    .def_property_readonly(
      "angular",
      [](BodyInstance const& body) { return body.motion().angular; })
    .def_property_readonly("mobility", &BodyInstance::mobility);

  py::class_<python_scene_snapshot_t>(module, "SceneSnapshot")
    .def(
      py::init([](std::uint64_t frame_id,
                  std::vector<BodyInstance> bodies) {
        return python_scene_snapshot_t {
          .value = std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
            .frame = FrameId {frame_id},
            .bodies = std::move(bodies),
          }),
        };
      }),
      py::arg("frame_id"),
      py::arg("bodies"))
    .def_property_readonly(
      "frame_id",
      [](python_scene_snapshot_t const& scene) {
        return scene.value->frame().value();
      })
    .def_property_readonly(
      "body_count",
      [](python_scene_snapshot_t const& scene) {
        return scene.value->bodyCount();
      })
    .def(
      "body_pose",
      [](python_scene_snapshot_t const& scene, std::uint64_t id) {
        return pose_vector(
          scene.value->body(EntityId {id}).frameFromBody());
      })
    .def(
      "body_linear_velocity",
      [](python_scene_snapshot_t const& scene, std::uint64_t id) {
        return scene.value->body(EntityId {id}).motion().linear;
      })
    .def(
      "body_angular_velocity",
      [](python_scene_snapshot_t const& scene, std::uint64_t id) {
        return scene.value->body(EntityId {id}).motion().angular;
      });

  py::class_<simulation_solver_stats_t>(module, "SolverStats")
    .def_readonly("iters", &simulation_solver_stats_t::iters)
    .def_readonly("converged", &simulation_solver_stats_t::converged)
    .def_readonly(
      "primal_residual", &simulation_solver_stats_t::primal_residual)
    .def_readonly(
      "dual_residual", &simulation_solver_stats_t::dual_residual);

  py::class_<python_simulation_result_t>(module, "SimulationResult")
    .def_readonly("snapshot", &python_simulation_result_t::snapshot)
    .def_property_readonly(
      "contacts",
      [](python_simulation_result_t const& result) {
        py::list values;
        for (contact_t const& contact : result.contacts) {
          values.append(contact_dict(contact));
        }
        return values;
      })
    .def_readonly("solver", &python_simulation_result_t::solver);

  py::class_<Simulator>(module, "Simulator")
    .def(
      py::init<simulation_config_t>(),
      py::arg("config") = simulation_config_t {})
    .def(
      "step",
      [](Simulator& simulator,
         python_scene_snapshot_t const& scene,
         Scalar dt) {
        py::gil_scoped_release release;
        return python_result(simulator.step(*scene.value, dt));
      },
      py::arg("scene"),
      py::arg("dt"))
    .def(
      "step_n",
      [](Simulator& simulator,
         python_scene_snapshot_t const& scene,
         Scalar dt,
         std::size_t count) {
        py::gil_scoped_release release;
        return python_result(simulator.step_n(*scene.value, dt, count));
      },
      py::arg("scene"),
      py::arg("dt"),
      py::arg("count"))
    .def("clear_warm_start", &Simulator::clearWarmStart);
}
