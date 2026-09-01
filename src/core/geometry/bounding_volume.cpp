#include <stacking_core/geometry/bounding_volume.hpp>

#include <stdexcept>
#include <utility>

namespace stacking_core {

BoundingVolume::BoundingVolume(
  bounding_volume_type_e type, Vector3 min, Vector3 max) noexcept
    : type_(type),
      minimum_(std::move(min)),
      maximum_(std::move(max)) {
}

BoundingVolume BoundingVolume::finite(Vector3 min, Vector3 max) {
  if (!min.allFinite() || !max.allFinite() ||
      (min.array() > max.array()).any()) {
    throw std::invalid_argument(
      "finite bounding volume must have finite ordered limits");
  }
  return BoundingVolume {
    bounding_volume_type_e::finite, std::move(min), std::move(max)};
}

BoundingVolume BoundingVolume::unbounded() noexcept {
  return BoundingVolume {
    bounding_volume_type_e::unbounded, Vector3::Zero(), Vector3::Zero()};
}

Vector3 const& BoundingVolume::minimum() const {
  if (!is_finite()) {
    throw std::logic_error("only a finite bounding volume has a minimum");
  }
  return minimum_;
}

Vector3 const& BoundingVolume::maximum() const {
  if (!is_finite()) {
    throw std::logic_error("only a finite bounding volume has a maximum");
  }
  return maximum_;
}

Vector3 BoundingVolume::center() const {
  return 0.5 * (minimum() + maximum());
}

Vector3 BoundingVolume::extent() const {
  return 0.5 * (maximum() - minimum());
}

BoundingVolume merge(
  BoundingVolume const& first, BoundingVolume const& second) {
  if (first.isUnbounded() || second.isUnbounded()) {
    return BoundingVolume::unbounded();
  }
  if (first.isEmpty()) {
    return second;
  }
  if (second.isEmpty()) {
    return first;
  }
  return BoundingVolume::finite(
    first.minimum().cwiseMin(second.minimum()),
    first.maximum().cwiseMax(second.maximum()));
}

BoundingVolume transformed(
  BoundingVolume const& volume, pose_t const& frame_from_local) {
  if (!is_valid(frame_from_local)) {
    throw std::invalid_argument("bounding-volume transform must be valid");
  }
  if (!volume.is_finite()) {
    return volume;
  }
  Vector3 const center_frame =
    transform_point(frame_from_local, volume.center());
  Matrix3 const absolute_rotation =
    frame_from_local.orientation.toRotationMatrix().cwiseAbs();
  Vector3 const extent_frame = absolute_rotation * volume.extent();
  return BoundingVolume::finite(
    center_frame - extent_frame, center_frame + extent_frame);
}

bool intersects(
  BoundingVolume const& first, BoundingVolume const& second) noexcept {
  if (first.isEmpty() || second.isEmpty()) {
    return false;
  }
  if (first.isUnbounded() || second.isUnbounded()) {
    return true;
  }
  return (first.minimum().array() <= second.maximum().array()).all() &&
    (second.minimum().array() <= first.maximum().array()).all();
}

}  // namespace stacking_core
