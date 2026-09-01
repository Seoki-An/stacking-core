#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <compare>
#include <cstdint>
#include <functional>
#include <limits>

namespace stacking_core {

using Scalar = double;

using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;
using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
using Matrix3 = Eigen::Matrix3d;
using Matrix36 = Eigen::Matrix<Scalar, 3, 6>;
using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;
using Matrix6X = Eigen::Matrix<Scalar, 6, Eigen::Dynamic>;
using Matrix3X = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;
using Quaternion = Eigen::Quaterniond;

template <typename Tag>
class Id {
public:
  using value_type = std::uint64_t;
  static constexpr value_type invalid_value =
    std::numeric_limits<value_type>::max();

  constexpr Id() noexcept = default;
  explicit constexpr Id(value_type value) noexcept: value_(value) {
  }

  [[nodiscard]] constexpr value_type value() const noexcept {
    return value_;
  }

  [[nodiscard]] constexpr bool valid() const noexcept {
    return value_ != invalid_value;
  }

  explicit constexpr operator bool() const noexcept {
    return valid();
  }

  auto operator<=>(Id const&) const = default;

private:
  value_type value_ = invalid_value;
};

struct entity_id_tag_t;
struct body_model_id_tag_t;
struct geometry_id_tag_t;
struct frame_id_tag_t;
struct link_id_tag_t;
struct joint_id_tag_t;

using EntityId = Id<entity_id_tag_t>;
using BodyModelId = Id<body_model_id_tag_t>;
using GeometryId = Id<geometry_id_tag_t>;
using FrameId = Id<frame_id_tag_t>;
using LinkId = Id<link_id_tag_t>;
using JointId = Id<joint_id_tag_t>;

}  // namespace stacking_core

namespace std {

template <typename Tag>
struct hash<stacking_core::Id<Tag>> {
  std::size_t operator()(stacking_core::Id<Tag> id) const noexcept {
    return hash<typename stacking_core::Id<Tag>::value_type> {}(id.value());
  }
};

}  // namespace std
