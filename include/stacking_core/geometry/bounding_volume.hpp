#pragma once

#include <stacking_core/transform.hpp>

namespace stacking_core {

enum class bounding_volume_type_e {
  empty,
  finite,
  unbounded,
};

class BoundingVolume {
public:
  BoundingVolume() = default;

  [[nodiscard]] static BoundingVolume finite(
    Vector3 min, Vector3 max);
  [[nodiscard]] static BoundingVolume unbounded() noexcept;

  [[nodiscard]] bounding_volume_type_e type() const noexcept {
    return type_;
  }

  [[nodiscard]] bool isEmpty() const noexcept {
    return type_ == bounding_volume_type_e::empty;
  }

  [[nodiscard]] bool is_finite() const noexcept {
    return type_ == bounding_volume_type_e::finite;
  }

  [[nodiscard]] bool isUnbounded() const noexcept {
    return type_ == bounding_volume_type_e::unbounded;
  }

  [[nodiscard]] Vector3 const& minimum() const;
  [[nodiscard]] Vector3 const& maximum() const;
  [[nodiscard]] Vector3 center() const;
  [[nodiscard]] Vector3 extent() const;

private:
  BoundingVolume(
    bounding_volume_type_e type, Vector3 min, Vector3 max) noexcept;

  bounding_volume_type_e type_ = bounding_volume_type_e::empty;
  Vector3 minimum_ = Vector3::Zero();
  Vector3 maximum_ = Vector3::Zero();
};

[[nodiscard]] BoundingVolume merge(
  BoundingVolume const& first, BoundingVolume const& second);
[[nodiscard]] BoundingVolume transformed(
  BoundingVolume const& volume, pose_t const& frame_from_local);
[[nodiscard]] bool intersects(
  BoundingVolume const& first, BoundingVolume const& second) noexcept;

}  // namespace stacking_core
