#include <stacking_core/collision/broad_phase.hpp>

#include <stacking_core/geometry/bounding_volume.hpp>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace stacking_core {

std::vector<collision_body_pair_t> brute_force_broad_phase(
  SceneView const& scene) {
  std::vector<collision_body_pair_t> candidates;
  for (std::size_t first_body_index = 0;
       first_body_index < scene.bodyCount();
       ++first_body_index) {
    BodyInstance const& first_body = scene.body(first_body_index);
    for (std::size_t second_body_index = first_body_index + 1;
         second_body_index < scene.bodyCount();
         ++second_body_index) {
      BodyInstance const& second_body = scene.body(second_body_index);
      candidates.push_back(collision_body_pair_t {
        .first = first_body.id(),
        .second = second_body.id(),
      });
    }
  }
  return candidates;
}

std::vector<collision_body_pair_t> bounding_volume_broad_phase(
  SceneView const& scene,
  Scalar margin) {
  if (!std::isfinite(margin) || margin < 0.0) {
    throw std::invalid_argument(
      "collision broad-phase margin must be finite and non-negative");
  }
  struct bounded_body_t {
    EntityId entity;
    BoundingVolume bounds;
  };

  std::vector<bounded_body_t> bodies;
  bodies.reserve(scene.bodyCount());
  for (std::size_t index = 0; index < scene.bodyCount(); ++index) {
    BodyInstance const& body = scene.body(index);
    bodies.push_back(bounded_body_t {
      .entity = body.id(),
      .bounds = transformed(
        body.model().boundingVolume(), body.frameFromBody()),
    });
  }

  std::vector<collision_body_pair_t> candidates;
  for (std::size_t first = 0; first < bodies.size(); ++first) {
    for (std::size_t second = first + 1; second < bodies.size(); ++second) {
      BoundingVolume const& bounds_1 = bodies[first].bounds;
      BoundingVolume const& bounds_2 = bodies[second].bounds;
      bool may_intersect = intersects(bounds_1, bounds_2);
      if (!may_intersect && bounds_1.is_finite() && bounds_2.is_finite()) {
        may_intersect =
          (bounds_1.minimum().array() <=
           (bounds_2.maximum().array() + margin)).all() &&
          (bounds_2.minimum().array() <=
           (bounds_1.maximum().array() + margin)).all();
      }
      if (may_intersect) {
        candidates.push_back(collision_body_pair_t {
          .first = bodies[first].entity,
          .second = bodies[second].entity,
        });
      }
    }
  }
  return candidates;
}

}  // namespace stacking_core
