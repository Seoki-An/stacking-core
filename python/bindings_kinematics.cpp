#include "bindings_common.hpp"

#include <stacking_core/io/urdf.hpp>
#include <stacking_core/kinematics.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nb = nanobind;

namespace stacking_core::python {

namespace {

struct python_kinematic_model_t {
  std::shared_ptr<KinematicModel const> value;
};

struct python_urdf_model_t {
  std::shared_ptr<UrdfModel> value;
};

nb::object optional_id(std::optional<std::size_t> const &value) {
  if (!value.has_value()) {
    return nb::none();
  }
  return nb::int_(*value);
}

} // namespace

void bind_kinematics(nb::module_ &module) {
  nb::class_<python_kinematic_model_t>(module, "KinematicModel")
      .def_prop_ro("link_count",
                   [](python_kinematic_model_t const &model) {
                     return model.value->linkCount();
                   })
      .def_prop_ro("joint_count",
                   [](python_kinematic_model_t const &model) {
                     return model.value->jointCount();
                   })
      .def_prop_ro("dof_count",
                   [](python_kinematic_model_t const &model) {
                     return model.value->degreeOfFreedomCount();
                   })
      .def_prop_ro("roots",
                   [](python_kinematic_model_t const &model) {
                     std::vector<std::uint64_t> result;
                     result.reserve(model.value->roots().size());
                     for (LinkId root : model.value->roots()) {
                       result.push_back(root.value());
                     }
                     return result;
                   })
      .def("find_link",
           [](python_kinematic_model_t const &model,
              std::string const &name) -> nb::object {
             auto const *link = model.value->findLink(name);
             if (link == nullptr) {
               return nb::none();
             }
             return nb::int_(link->id.value());
           })
      .def("find_joint",
           [](python_kinematic_model_t const &model,
              std::string const &name) -> nb::object {
             auto const *joint = model.value->findJoint(name);
             if (joint == nullptr) {
               return nb::none();
             }
             return nb::int_(joint->id.value());
           })
      .def("dof_index",
           [](python_kinematic_model_t const &model, std::uint64_t joint_id) {
             return optional_id(
                 model.value->degreeOfFreedomIndex(JointId{joint_id}));
           });

  nb::class_<python_urdf_model_t>(module, "UrdfModel")
      .def_prop_ro(
          "name",
          [](python_urdf_model_t const &model) { return model.value->name(); })
      .def_prop_ro("kinematics",
                   [](python_urdf_model_t const &model) {
                     return python_kinematic_model_t{
                         .value = model.value->kinematicsPtr(),
                     };
                   })
      .def("body_model",
           [](python_urdf_model_t const &model, std::uint64_t link_id) {
             return model.value->bodyModelPtr(LinkId{link_id});
           });

  module.def(
      "load_urdf_model",
      [](std::string const &path) {
        return python_urdf_model_t{
            .value = std::make_shared<UrdfModel>(
                load_urdf_model(std::filesystem::path{path})),
        };
      },
      nb::arg("path"));

  nb::class_<KinematicState>(module, "KinematicState")
      .def(nb::new_([](std::uint64_t frame_id,
                       python_kinematic_model_t const &model) {
             return KinematicState{FrameId{frame_id}, model.value};
           }),
           nb::arg("frame_id"), nb::arg("model"))
      .def_prop_ro(
          "frame_id",
          [](KinematicState const &state) { return state.frame().value(); })
      .def_prop_ro("model",
                   [](KinematicState const &state) {
                     return python_kinematic_model_t{.value = state.modelPtr()};
                   })
      .def_prop_rw(
          "positions",
          [](KinematicState const &state) { return state.positions(); },
          [](KinematicState &state, Eigen::VectorXd const &positions) {
            state.setPositions(positions);
          })
      .def("set_positions",
           [](KinematicState &state, Eigen::VectorXd const &positions) {
             state.setPositions(positions);
           })
      .def("set_frame_from_root",
           [](KinematicState &state, std::uint64_t root_id,
              array_t const &pose) {
             state.setFrameFromRoot(LinkId{root_id},
                                    pose_from_array(pose, "frame_from_root"));
           })
      .def("frame_from_root",
           [](KinematicState const &state, std::uint64_t root_id) {
             return pose_vector(state.frameFromRoot(LinkId{root_id}));
           })
      .def("link_pose",
           [](KinematicState const &state, std::uint64_t link_id) {
             return pose_vector(
                 forward_kinematics(state).frameFromLink(LinkId{link_id}));
           })
      .def("positions_within_limits", &KinematicState::positionsWithinLimits,
           nb::arg("tol") = 0.0);
}

} // namespace stacking_core::python
