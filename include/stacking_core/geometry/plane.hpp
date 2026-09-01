#pragma once

#include <stacking_core/geometry/geometry.hpp>

namespace stacking_core {

// A plane occupies local z = 0 and its local normal is +Z.
struct plane_geometry_config_t {
  geometry_properties_t properties;
};

class PlaneGeometry final: public Geometry {
public:
  explicit PlaneGeometry(plane_geometry_config_t config);

  [[nodiscard]] geometry_type_e type() const noexcept override {
    return geometry_type_e::plane;
  }

  [[nodiscard]] Vector3 center(pose_t const& frame_from_body) const;
  [[nodiscard]] Vector3 normal(pose_t const& frame_from_body) const;
  [[nodiscard]] Scalar signedDistance(
    pose_t const& frame_from_body, Vector3 const& point_frame) const;
};

}  // namespace stacking_core
