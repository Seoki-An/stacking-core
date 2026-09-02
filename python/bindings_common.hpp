#pragma once

#include <stacking_core/scene.hpp>
#include <stacking_core/transform.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace stacking_core::python {

namespace nb = nanobind;

using pose_vector_t = Eigen::Matrix<Scalar, 7, 1>;
using array_t =
    nb::ndarray<nb::numpy, Scalar const, nb::c_contig, nb::device::cpu>;

inline Vector3 vector3_from_array(array_t const &value, char const *name) {
  if (value.ndim() != 1 || value.shape(0) != 3) {
    throw std::invalid_argument(std::string{name} + " must have shape (3,)");
  }
  Scalar const *data = value.data();
  return Vector3{data[0], data[1], data[2]};
}

inline Matrix3 matrix3_from_array(array_t const &value, char const *name) {
  if (value.ndim() != 2 || value.shape(0) != 3 || value.shape(1) != 3) {
    throw std::invalid_argument(std::string{name} + " must have shape (3, 3)");
  }
  Scalar const *data = value.data();
  Matrix3 result;
  for (Eigen::Index row = 0; row < 3; ++row) {
    for (Eigen::Index col = 0; col < 3; ++col) {
      result(row, col) = data[row * 3 + col];
    }
  }
  return result;
}

inline Matrix6 matrix6_from_array(array_t const &value, char const *name) {
  if (value.ndim() != 2 || value.shape(0) != 6 || value.shape(1) != 6) {
    throw std::invalid_argument(std::string{name} + " must have shape (6, 6)");
  }
  Scalar const *data = value.data();
  Matrix6 result;
  for (Eigen::Index row = 0; row < 6; ++row) {
    for (Eigen::Index col = 0; col < 6; ++col) {
      result(row, col) = data[row * 6 + col];
    }
  }
  return result;
}

inline pose_t pose_from_array(array_t const &value, char const *name = "pose") {
  if (value.ndim() != 1 || value.shape(0) != 7) {
    throw std::invalid_argument(std::string{name} +
                                " must contain [x, y, z, qx, qy, qz, qw]");
  }
  Scalar const *data = value.data();
  return pose_t{
      Vector3{data[0], data[1], data[2]},
      Quaternion{data[6], data[3], data[4], data[5]},
  };
}

inline pose_vector_t pose_vector(pose_t const &pose) {
  pose_vector_t value;
  value << pose.position, pose.orientation.coeffs();
  return value;
}

inline std::optional<pose_t> optional_pose_from_object(nb::object const &value,
                                                       char const *name) {
  if (value.is_none()) {
    return std::nullopt;
  }
  return pose_from_array(nb::cast<array_t>(value), name);
}

inline nb::object optional_pose_object(std::optional<pose_t> const &value) {
  if (!value.has_value()) {
    return nb::none();
  }
  return nb::cast(pose_vector(*value));
}

inline std::vector<EntityId>
entity_ids(std::vector<std::uint64_t> const &values) {
  std::vector<EntityId> result;
  result.reserve(values.size());
  for (std::uint64_t value : values) {
    result.emplace_back(value);
  }
  return result;
}

inline std::vector<std::uint64_t>
python_entity_ids(std::span<EntityId const> values) {
  std::vector<std::uint64_t> result;
  result.reserve(values.size());
  for (EntityId value : values) {
    result.push_back(value.value());
  }
  return result;
}

struct python_scene_snapshot_t {
  std::shared_ptr<SceneSnapshot const> value;
};

void bind_simulation(nb::module_ &module);
void bind_scene(nb::module_ &module);
void bind_kinematics(nb::module_ &module);
void bind_planner_types(nb::module_ &module);
void bind_planner_grasp(nb::module_ &module);
void bind_planner_motion(nb::module_ &module);
void bind_planner_pick_place(nb::module_ &module);

} // namespace stacking_core::python
