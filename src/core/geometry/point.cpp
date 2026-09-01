#include <stacking_core/geometry/point.hpp>

#include <utility>

namespace stacking_core {

PointGeometry::PointGeometry(point_geometry_config_t config)
    : Geometry(
        std::move(config.properties),
        BoundingVolume::finite(Vector3::Zero(), Vector3::Zero())) {
}

Vector3 PointGeometry::point(pose_t const& frame_from_body) const {
  pose_t const frame_from_point = compose(frame_from_body, bodyFromGeometry());
  return frame_from_point.position;
}

}  // namespace stacking_core
