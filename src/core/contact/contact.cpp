#include <stacking_core/contact/contact.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace stacking_core {

contact_frame_t make_contact_frame(Vector3 const& normal_frame) {
  Scalar const normal_norm = normal_frame.norm();
  if (!normal_frame.allFinite() || !std::isfinite(normal_norm) ||
      normal_norm <= 0.0) {
    throw std::invalid_argument("contact normal must be finite and non-zero");
  }

  Vector3 const normal = normal_frame / normal_norm;
  Vector3 const tangent_first = normal.unitOrthogonal().normalized();
  Vector3 const tangent_second = normal.cross(tangent_first).normalized();
  return contact_frame_t {
    .tangent_first = tangent_first,
    .tangent_second = tangent_second,
    .normal = normal,
  };
}

Scalar combine_friction(
  Scalar first_friction, Scalar second_friction) {
  if (!std::isfinite(first_friction) || first_friction < 0.0 ||
      !std::isfinite(second_friction) || second_friction < 0.0) {
    throw std::invalid_argument(
      "material friction must be finite and non-negative");
  }
  Scalar const sum = first_friction + second_friction;
  if (sum == 0.0) {
    return 0.0;
  }
  return 2.0 * first_friction * second_friction / sum;
}

contact_t make_contact(SceneSnapshot const& scene, contact_feature_t feature) {
  if (!feature.pair.first.valid() || !feature.pair.second.valid() ||
      feature.pair.first.entity == feature.pair.second.entity) {
    throw std::invalid_argument("contact pair must contain distinct valid entities");
  }
  if (!std::isfinite(feature.gap) || !feature.point_first.allFinite() ||
      !feature.point_second.allFinite()) {
    throw std::invalid_argument("contact feature values must be finite");
  }

  contact_frame_t const frame = make_contact_frame(feature.normal);
  feature.normal = frame.normal;
  Geometry const& first_geometry = scene.body(feature.pair.first.entity)
    .model().geometry(feature.pair.first.geometry);
  Geometry const& second_geometry = scene.body(feature.pair.second.entity)
    .model().geometry(feature.pair.second.geometry);
  return contact_t {
    .feature = std::move(feature),
    .frame = frame,
    .friction = combine_friction(
      first_geometry.material().friction,
      second_geometry.material().friction),
  };
}

}  // namespace stacking_core
