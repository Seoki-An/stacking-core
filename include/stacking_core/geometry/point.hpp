#pragma once

#include <stacking_core/geometry/geometry.hpp>

namespace stacking_core {

// A point occupies the origin of its local geometry frame.
struct point_geometry_config_t {
  geometry_properties_t properties;
};

class PointGeometry final: public Geometry {
public:
  explicit PointGeometry(point_geometry_config_t config);

  [[nodiscard]] geometry_type_e type() const noexcept override {
    return geometry_type_e::point;
  }

  [[nodiscard]] Vector3 point(pose_t const& frame_from_body) const;
};

}  // namespace stacking_core
