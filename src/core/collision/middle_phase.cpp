#include <stacking_core/collision/middle_phase.hpp>

#include <stacking_core/geometry/plane.hpp>

#include <cmath>
#include <stdexcept>

namespace stacking_core {

bool supports_collision_pair(
  geometry_type_e first, geometry_type_e second) noexcept {
  return first != geometry_type_e::plane || second != geometry_type_e::plane;
}

namespace {

bool plane_intersects(
  PlaneGeometry const& plane,
  BodyInstance const& plane_body,
  BoundingVolume const& other_bounds,
  Scalar margin) {
  if (other_bounds.isEmpty()) {
    return false;
  }
  if (other_bounds.isUnbounded()) {
    return true;
  }
  Vector3 const normal = plane.normal(plane_body.frameFromBody());
  Vector3 const plane_center = plane.center(plane_body.frameFromBody());
  Scalar const center_distance =
    normal.dot(other_bounds.center() - plane_center);
  Scalar const radius = normal.cwiseAbs().dot(other_bounds.extent());
  return center_distance <= radius + margin + 1e-12;
}

bool bounds_may_intersect(
  Geometry const& first_geometry,
  BodyInstance const& first_body,
  BoundingVolume const& first_bounds,
  Geometry const& second_geometry,
  BodyInstance const& second_body,
  BoundingVolume const& second_bounds,
  Scalar margin) {
  if (first_geometry.type() == geometry_type_e::plane) {
    return plane_intersects(
      static_cast<PlaneGeometry const&>(first_geometry),
      first_body,
      second_bounds,
      margin);
  }
  if (second_geometry.type() == geometry_type_e::plane) {
    return plane_intersects(
      static_cast<PlaneGeometry const&>(second_geometry),
      second_body,
      first_bounds,
      margin);
  }
  if (intersects(first_bounds, second_bounds)) {
    return true;
  }
  if (!first_bounds.is_finite() || !second_bounds.is_finite()) {
    return false;
  }
  return
    (first_bounds.minimum().array() <=
     (second_bounds.maximum().array() + margin)).all() &&
    (second_bounds.minimum().array() <=
     (first_bounds.maximum().array() + margin)).all();
}

void validate_body_pair(
  SceneView const& scene,
  collision_body_pair_t const& body_pair,
  BodyInstance const*& first_body,
  BodyInstance const*& second_body) {
  if (body_pair.first == body_pair.second) {
    throw std::invalid_argument(
      "collision middle-phase body pair must contain distinct entities");
  }
  first_body = scene.findBody(body_pair.first);
  second_body = scene.findBody(body_pair.second);
  if (first_body == nullptr || second_body == nullptr) {
    throw std::invalid_argument(
      "collision middle-phase body pair must belong to the scene view");
  }
}

}  // namespace

std::vector<collision_pair_t> brute_force_middle_phase(
  SceneView const& scene,
  std::span<collision_body_pair_t const> body_pairs) {
  std::vector<collision_pair_t> candidates;
  for (collision_body_pair_t const& body_pair : body_pairs) {
    BodyInstance const* first_body = nullptr;
    BodyInstance const* second_body = nullptr;
    validate_body_pair(scene, body_pair, first_body, second_body);

    for (std::size_t first_geometry_index = 0;
         first_geometry_index < first_body->model().geometryCount();
         ++first_geometry_index) {
      Geometry const& first_geometry =
        first_body->model().geometry(first_geometry_index);
      for (std::size_t second_geometry_index = 0;
           second_geometry_index < second_body->model().geometryCount();
           ++second_geometry_index) {
        Geometry const& second_geometry =
          second_body->model().geometry(second_geometry_index);
        if (!supports_collision_pair(
              first_geometry.type(), second_geometry.type())) {
          continue;
        }
        candidates.push_back(collision_pair_t {
          .first = geometry_instance_id_t {
            .entity = first_body->id(),
            .geometry = first_geometry.id(),
          },
          .second = geometry_instance_id_t {
            .entity = second_body->id(),
            .geometry = second_geometry.id(),
          },
        });
      }
    }
  }
  return candidates;
}

std::vector<collision_pair_t> bounding_volume_middle_phase(
  SceneView const& scene,
  std::span<collision_body_pair_t const> body_pairs,
  Scalar margin) {
  if (!std::isfinite(margin) || margin < 0.0) {
    throw std::invalid_argument(
      "collision middle-phase margin must be finite and non-negative");
  }
  std::vector<collision_pair_t> candidates;
  for (collision_body_pair_t const& body_pair : body_pairs) {
    BodyInstance const* first_body = nullptr;
    BodyInstance const* second_body = nullptr;
    validate_body_pair(scene, body_pair, first_body, second_body);

    for (std::size_t first_geometry_index = 0;
         first_geometry_index < first_body->model().geometryCount();
         ++first_geometry_index) {
      Geometry const& first_geometry =
        first_body->model().geometry(first_geometry_index);
      BoundingVolume const first_bounds = transformed(
        first_geometry.boundingVolume(),
        first_body->frameFromGeometry(first_geometry.id()));
      for (std::size_t second_geometry_index = 0;
           second_geometry_index < second_body->model().geometryCount();
           ++second_geometry_index) {
        Geometry const& second_geometry =
          second_body->model().geometry(second_geometry_index);
        if (!supports_collision_pair(
              first_geometry.type(), second_geometry.type())) {
          continue;
        }
        BoundingVolume const second_bounds = transformed(
          second_geometry.boundingVolume(),
          second_body->frameFromGeometry(second_geometry.id()));
        if (!bounds_may_intersect(
              first_geometry,
              *first_body,
              first_bounds,
              second_geometry,
              *second_body,
              second_bounds,
              margin)) {
          continue;
        }
        candidates.push_back(collision_pair_t {
          .first = geometry_instance_id_t {
            .entity = first_body->id(),
            .geometry = first_geometry.id(),
          },
          .second = geometry_instance_id_t {
            .entity = second_body->id(),
            .geometry = second_geometry.id(),
          },
        });
      }
    }
  }
  return candidates;
}

}  // namespace stacking_core
