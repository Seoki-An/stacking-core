#pragma once

#include <stacking_core/geometry/geometry.hpp>

#include <limits>
#include <type_traits>
#include <vector>

namespace stacking_core {

struct dsf_vert_geometry_config_t {
  geometry_properties_t properties;
  // Nodes are expressed in the configured local geometry frame. Construction
  // recenters them and adjusts body_from_geometry by the same offset.
  Matrix3X nodes;
  int sharpness = 20;
  Scalar epsilon = std::numeric_limits<Scalar>::epsilon();
};

struct support_t {
  Scalar h = 0.0;
  Vector3 s = Vector3::Zero();
};

struct diffable_support_t: support_t {
  // ds/dx, with s and x both expressed in the query frame.
  Matrix3 ds_dx = Matrix3::Zero();

  // ds/dq for a right perturbation of frame_from_body:
  // position' = position + delta_position_frame
  // rotation' = rotation * Exp(delta_rotation_body).
  Matrix36 ds_dq = Matrix36::Zero();
};

class DsfVertGeometry final: public Geometry {
public:
  explicit DsfVertGeometry(dsf_vert_geometry_config_t config);

  [[nodiscard]] geometry_type_e type() const noexcept override {
    return geometry_type_e::dsf_vert;
  }

  [[nodiscard]] Matrix3X const& nodes() const noexcept {
    return nodes_;
  }

  [[nodiscard]] int sharpness() const noexcept {
    return sharpness_;
  }

  [[nodiscard]] Scalar epsilon() const noexcept {
    return epsilon_;
  }

  [[nodiscard]] support_t support(
    Vector3 const& dir_frame, pose_t const& frame_from_body) const;

  [[nodiscard]] diffable_support_t diffable_support(
    Vector3 const& dir_frame, pose_t const& frame_from_body) const;

private:
  struct prepared_config_t;

  explicit DsfVertGeometry(prepared_config_t config);
  static prepared_config_t prepare(dsf_vert_geometry_config_t config);

  template <bool with_derivatives>
  [[nodiscard]] std::conditional_t<
    with_derivatives, diffable_support_t, support_t>
  support_impl(
    Vector3 const& dir_frame, pose_t const& frame_from_body) const;

  Matrix3X nodes_;
  int sharpness_;
  Scalar epsilon_;
  Scalar coefficient_threshold_;
  std::vector<Matrix3> node_outer_products_;
};

}  // namespace stacking_core
