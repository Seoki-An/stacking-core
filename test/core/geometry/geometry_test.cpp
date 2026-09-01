#include <stacking_core/geometry.hpp>

#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>

namespace {

using stacking_core::Scalar;
using stacking_core::Vector3;

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("geometry test requirement failed");
  }
}

bool near(Vector3 const& lhs, Vector3 const& rhs) {
  return (lhs - rhs).norm() < 1e-12;
}

}  // namespace

int main() {
  using namespace stacking_core;

  pose_t const body_from_point {Vector3 {1.0, 0.0, 0.0}, Quaternion::Identity()};
  PointGeometry point_geometry {{
    .properties = {
      .id = GeometryId {1},
      .body_from_geometry = body_from_point,
      .material = material_t {},
    },
  }};
  pose_t const world_from_body {Vector3 {0.0, 2.0, 0.0}, Quaternion::Identity()};
  require(point_geometry.type() == geometry_type_e::point);
  require(near(point_geometry.point(world_from_body), Vector3 {1.0, 2.0, 0.0}));

  Quaternion const plane_rotation {
    Eigen::AngleAxisd(std::numbers::pi_v<Scalar> / 2.0, Vector3::UnitY())};
  PlaneGeometry plane_geometry {{
    .properties = {
      .id = GeometryId {2},
      .body_from_geometry = pose_t {Vector3 {0.0, 0.0, 1.0}, plane_rotation},
      .material = material_t {.friction = 0.5},
    },
  }};
  require(plane_geometry.type() == geometry_type_e::plane);
  require(near(plane_geometry.center(pose_t {}), Vector3 {0.0, 0.0, 1.0}));
  require(near(plane_geometry.normal(pose_t {}), Vector3::UnitX()));
  require(std::abs(
            plane_geometry.signedDistance(pose_t {}, Vector3 {2.0, 0.0, 1.0}) -
            2.0) < 1e-12);
  require(std::abs(plane_geometry.material().friction - 0.5) < 1e-12);

  geometry_config_t point_config = point_geometry_config_t {
    .properties = {
      .id = GeometryId {3},
      .body_from_geometry = pose_t {},
      .material = material_t {},
    },
  };
  std::unique_ptr<Geometry> geometry = make_geometry(std::move(point_config));
  require(geometry->id() == GeometryId {3});
  require(geometry->type() == geometry_type_e::point);

  bool rejected_invalid_id = false;
  try {
    PointGeometry const invalid_geometry {point_geometry_config_t {}};
    (void)invalid_geometry;
  } catch (std::invalid_argument const&) {
    rejected_invalid_id = true;
  }
  require(rejected_invalid_id);

  bool rejected_friction = false;
  try {
    PlaneGeometry const invalid_geometry {{
      .properties = {
        .id = GeometryId {4},
        .body_from_geometry = pose_t {},
        .material = material_t {.friction = -1.0},
      },
    }};
    (void)invalid_geometry;
  } catch (std::invalid_argument const&) {
    rejected_friction = true;
  }
  require(rejected_friction);
}
