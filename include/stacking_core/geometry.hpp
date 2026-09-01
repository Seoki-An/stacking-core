#pragma once

#include <stacking_core/geometry/dsf_vert.hpp>
#include <stacking_core/geometry/bounding_volume.hpp>
#include <stacking_core/geometry/geometry.hpp>
#include <stacking_core/geometry/plane.hpp>
#include <stacking_core/geometry/point.hpp>

#include <memory>
#include <variant>

namespace stacking_core {

using geometry_config_t = std::variant<
  point_geometry_config_t,
  plane_geometry_config_t,
  dsf_vert_geometry_config_t>;

[[nodiscard]] std::unique_ptr<Geometry> make_geometry(geometry_config_t config);

}  // namespace stacking_core
