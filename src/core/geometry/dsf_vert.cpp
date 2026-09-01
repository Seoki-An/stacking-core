#include <stacking_core/geometry/dsf_vert.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace stacking_core {

struct DsfVertGeometry::prepared_config_t {
  geometry_properties_t properties;
  BoundingVolume bounding_volume;
  Matrix3X nodes;
  int sharpness;
  Scalar epsilon;
};

DsfVertGeometry::DsfVertGeometry(dsf_vert_geometry_config_t config)
    : DsfVertGeometry(prepare(std::move(config))) {
}

DsfVertGeometry::DsfVertGeometry(prepared_config_t config)
    : Geometry(
        std::move(config.properties), std::move(config.bounding_volume)),
      nodes_(std::move(config.nodes)),
      sharpness_(config.sharpness),
      epsilon_(config.epsilon),
      coefficient_threshold_(std::pow(epsilon_, 1.0 / sharpness_)) {
  node_outer_products_.reserve(static_cast<std::size_t>(nodes_.cols()));
  for (Eigen::Index i = 0; i < nodes_.cols(); ++i) {
    node_outer_products_.push_back(
      nodes_.col(i) * nodes_.col(i).transpose());
  }
}

DsfVertGeometry::prepared_config_t DsfVertGeometry::prepare(
  dsf_vert_geometry_config_t config) {
  if (config.nodes.cols() == 0) {
    throw std::invalid_argument("DSF-Vert geometry requires at least one node");
  }
  if (!config.nodes.allFinite()) {
    throw std::invalid_argument("DSF-Vert nodes must be finite");
  }
  if (config.sharpness < 2) {
    throw std::invalid_argument("DSF-Vert sharpness must be at least 2");
  }
  if (!std::isfinite(config.epsilon) || config.epsilon <= 0.0 ||
      config.epsilon >= 1.0) {
    throw std::invalid_argument("DSF-Vert epsilon must be finite and between 0 and 1");
  }

  Vector3 const center = config.nodes.rowwise().mean();
  config.nodes.colwise() -= center;

  auto support_height = [&](Vector3 const& dir) {
    Eigen::VectorXd a =
      (config.nodes.transpose() * dir).cwiseMax(0.0);
    Scalar const max_a = a.maxCoeff();
    if (max_a <= 0.0) {
      return Scalar {0.0};
    }
    a /= max_a;
    Scalar pow_a_sum = 0.0;
    for (Eigen::Index i = 0; i < a.size(); ++i) {
      pow_a_sum += std::pow(a[i], config.sharpness);
    }
    return max_a * std::pow(pow_a_sum, 1.0 / config.sharpness);
  };
  Vector3 min;
  Vector3 max;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    Vector3 dir = Vector3::Zero();
    dir[axis] = 1.0;
    max[axis] = support_height(dir);
    min[axis] = -support_height(-dir);
  }
  BoundingVolume const bounding_volume =
    BoundingVolume::finite(min, max);

  config.properties.body_from_geometry = compose(
    config.properties.body_from_geometry,
    pose_t {center, Quaternion::Identity()});

  return prepared_config_t {
    .properties = std::move(config.properties),
    .bounding_volume = bounding_volume,
    .nodes = std::move(config.nodes),
    .sharpness = config.sharpness,
    .epsilon = config.epsilon,
  };
}

support_t DsfVertGeometry::support(
  Vector3 const& dir_frame, pose_t const& frame_from_body) const {
  return support_impl<false>(dir_frame, frame_from_body);
}

diffable_support_t DsfVertGeometry::diffable_support(
  Vector3 const& dir_frame, pose_t const& frame_from_body) const {
  return support_impl<true>(dir_frame, frame_from_body);
}

template <bool with_derivatives>
std::conditional_t<with_derivatives, diffable_support_t, support_t>
DsfVertGeometry::support_impl(
  Vector3 const& dir_frame, pose_t const& frame_from_body) const {
  using out_t =
    std::conditional_t<with_derivatives, diffable_support_t, support_t>;
  if (!dir_frame.allFinite() || dir_frame.squaredNorm() <= 0.0) {
    throw std::invalid_argument("support direction must be finite and non-zero");
  }

  pose_t const frame_from_geom =
    compose(frame_from_body, bodyFromGeometry());
  Matrix3 const R_fb =
    frame_from_body.orientation.toRotationMatrix();
  Matrix3 const R_bg =
    bodyFromGeometry().orientation.toRotationMatrix();
  Matrix3 const R_fg = frame_from_geom.orientation.toRotationMatrix();
  Vector3 const dir_local =
    frame_from_geom.orientation.conjugate() * dir_frame;

  auto body_pose_jac = [&](Vector3 const& point_frame,
                           Matrix3 const& ds_dx) {
    Matrix36 J_geom = Matrix36::Zero();
    J_geom.leftCols<3>() = Matrix3::Identity();
    J_geom.rightCols<3>() =
      -skew(point_frame - frame_from_geom.position) * R_fg +
      ds_dx * R_fg * skew(dir_local);

    Matrix6 J_geom_body = Matrix6::Identity();
    J_geom_body.block<3, 3>(0, 3) =
      -R_fb * skew(bodyFromGeometry().position);
    J_geom_body.block<3, 3>(3, 3) = R_bg.transpose();
    return (J_geom * J_geom_body).eval();
  };

  Eigen::VectorXd a =
    (nodes_.transpose() * dir_local).cwiseMax(0.0);
  Scalar const max_a = a.maxCoeff();
  if (!std::isfinite(max_a) || max_a <= 0.0) {
    out_t out;
    out.h = frame_from_geom.position.dot(dir_frame);
    out.s = frame_from_geom.position;
    if constexpr (with_derivatives) {
      out.ds_dq = body_pose_jac(out.s, out.ds_dx);
    }
    return out;
  }
  a /= max_a;

  Vector3 s_local = Vector3::Zero();
  Matrix3 ds_dx_local = Matrix3::Zero();
  Scalar pow_a_sum = 0.0;
  for (Eigen::Index i = 0; i < nodes_.cols(); ++i) {
    Scalar const a_i = a[i];
    if (a_i <= coefficient_threshold_) {
      continue;
    }
    Scalar pow_a = 0.0;
    if constexpr (with_derivatives) {
      Scalar const pow_a_hess = std::pow(a_i, sharpness_ - 2);
      ds_dx_local +=
        pow_a_hess * node_outer_products_[static_cast<std::size_t>(i)];
      pow_a = pow_a_hess * a_i;
    } else {
      pow_a = std::pow(a_i, sharpness_ - 1);
    }
    s_local += pow_a * nodes_.col(i);
    pow_a_sum += pow_a * a_i;
  }

  if (!std::isfinite(pow_a_sum) || pow_a_sum <= 0.0) {
    out_t out;
    out.h = frame_from_geom.position.dot(dir_frame);
    out.s = frame_from_geom.position;
    if constexpr (with_derivatives) {
      out.ds_dq = body_pose_jac(out.s, out.ds_dx);
    }
    return out;
  }

  Scalar const h = std::pow(pow_a_sum, 1.0 / sharpness_);
  Scalar const h_scale = h / pow_a_sum;
  s_local *= h_scale;
  if constexpr (with_derivatives) {
    ds_dx_local *= (sharpness_ - 1) * h_scale / max_a;
    ds_dx_local -=
      (sharpness_ - 1) / (h * max_a) * s_local * s_local.transpose();
  }

  out_t out;
  out.s = transform_point(frame_from_geom, s_local);
  out.h = out.s.dot(dir_frame);
  if constexpr (with_derivatives) {
    out.ds_dx = R_fg * ds_dx_local * R_fg.transpose();
    out.ds_dq = body_pose_jac(out.s, out.ds_dx);
  }
  return out;
}

}  // namespace stacking_core
