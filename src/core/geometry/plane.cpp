#include <stacking_core/geometry/plane.hpp>

#include <utility>

namespace stacking_core {

PlaneGeometry::PlaneGeometry(plane_geometry_config_t config)
    : Geometry(
        std::move(config.properties), BoundingVolume::unbounded()) {
}

Vector3 PlaneGeometry::center(pose_t const& frame_from_body) const {
  pose_t const frame_from_plane = compose(frame_from_body, bodyFromGeometry());
  return frame_from_plane.position;
}

Vector3 PlaneGeometry::normal(pose_t const& frame_from_body) const {
  pose_t const frame_from_plane = compose(frame_from_body, bodyFromGeometry());
  return transform_vector(frame_from_plane, Vector3::UnitZ());
}

Scalar PlaneGeometry::signedDistance(
  pose_t const& frame_from_body, Vector3 const& point_frame) const {
  return normal(frame_from_body).dot(point_frame - center(frame_from_body));
}

}  // namespace stacking_core
