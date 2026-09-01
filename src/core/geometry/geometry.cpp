#include <stacking_core/geometry.hpp>

#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace stacking_core {

Geometry::Geometry(
  geometry_properties_t properties, BoundingVolume bounding_volume)
    : id_(properties.id),
      body_from_geometry_(std::move(properties.body_from_geometry)),
      material_(properties.material),
      bounding_volume_(std::move(bounding_volume)) {
  if (!id_.valid()) {
    throw std::invalid_argument("geometry ID must be valid");
  }
  if (!is_valid(body_from_geometry_)) {
    throw std::invalid_argument("body_from_geometry must be a valid pose");
  }
  if (!std::isfinite(material_.friction) || material_.friction < 0.0) {
    throw std::invalid_argument("geometry friction must be finite and non-negative");
  }
}

std::unique_ptr<Geometry> make_geometry(geometry_config_t config) {
  return std::visit(
    [](auto typed_config) -> std::unique_ptr<Geometry> {
      using Config = decltype(typed_config);
      if constexpr (std::is_same_v<Config, point_geometry_config_t>) {
        return std::make_unique<PointGeometry>(std::move(typed_config));
      } else if constexpr (std::is_same_v<Config, plane_geometry_config_t>) {
        return std::make_unique<PlaneGeometry>(std::move(typed_config));
      } else {
        return std::make_unique<DsfVertGeometry>(std::move(typed_config));
      }
    },
    std::move(config));
}

}  // namespace stacking_core
