#include <stacking_core/collision/narrow_phase.hpp>

#include <stacking_core/collision/middle_phase.hpp>
#include <stacking_core/geometry/dsf_vert.hpp>
#include <stacking_core/geometry/plane.hpp>
#include <stacking_core/geometry/point.hpp>
#include <stacking_core/optimization/truncated_conjugate_gradient.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace stacking_core {
  namespace {

    struct geometry_ref_t {
      BodyInstance const& body;
      Geometry const& geometry;
    };

    geometry_ref_t find_geometry(
      SceneSnapshot const& scene, geometry_instance_id_t id) {
      if (!id.valid()) {
        throw std::invalid_argument(
          "collision geometry instance ID must be valid");
      }
      BodyInstance const& body = scene.body(id.entity);
      return geometry_ref_t {
        .body = body,
        .geometry = body.model().geometry(id.geometry),
      };
    }

    Vector3 geometry_center(geometry_ref_t const& ref) {
      return ref.body.frameFromGeometry(ref.geometry.id()).position;
    }

    template <typename Evaluate>
    Vector3 minimize_dir(
      Vector3 init, Scalar min_acceptance_gain_ratio,
      narrow_phase_config_t const& config, Evaluate&& eval) {
      Scalar const init_norm = init.norm();
      Vector3 dir = Vector3::UnitZ();
      if (init.allFinite() && init_norm > 0.0) {
        dir = init / init_norm;
      }

      Scalar tr_radius = config.tr_radius_init;
      for (int iter = 0; iter < config.max_dir_iters; ++iter) {
        diffable_support_t const cur = eval(dir);
        Vector3 const grad_t = cur.s - cur.s.dot(dir) * dir;
        Scalar const grad_norm = grad_t.norm();
        if (
          !std::isfinite(cur.h) || !grad_t.allFinite() ||
          !cur.ds_dx.allFinite()) {
          throw std::runtime_error(
            "collision direction optimization became non-finite");
        }
        if (grad_norm <= config.dir_tol) {
          break;
        }

        Eigen::Matrix<Scalar, 3, 2> V_t;
        V_t.col(0) = dir.unitOrthogonal().normalized();
        V_t.col(1) = dir.cross(V_t.col(0)).normalized();
        Eigen::Vector2d const grad_proj = V_t.transpose() * grad_t;
        Eigen::Matrix2d const hess_proj = V_t.transpose() * cur.ds_dx * V_t -
          cur.s.dot(dir) * Eigen::Matrix2d::Identity();
        auto const [step, is_boundary] = truncated_conjugate_gradient(
          -grad_proj, hess_proj, config.tr_subproblem_tol, tr_radius);
        if (step.squaredNorm() <= std::numeric_limits<Scalar>::epsilon()) {
          break;
        }

        Vector3 const dir_new = (dir + V_t * step).normalized();
        diffable_support_t const next = eval(dir_new);
        Scalar const f_model =
          cur.h + grad_proj.dot(step) + 0.5 * step.dot(hess_proj * step);
        Scalar const pred_reduction = cur.h - f_model;
        Scalar rho = -std::numeric_limits<Scalar>::infinity();
        if (
          std::isfinite(next.h) && std::isfinite(pred_reduction) &&
          pred_reduction > 0.0) {
          rho = (cur.h - next.h) / pred_reduction;
        }

        if (rho > min_acceptance_gain_ratio) {
          dir = dir_new;
        }
        if (rho < config.gain_ratio_lower_thresh) {
          tr_radius *= config.tr_radius_reduction_rate;
        } else if (rho > config.gain_ratio_upper_thresh && is_boundary) {
          tr_radius =
            std::min(config.tr_radius_expansion_rate * tr_radius, Scalar {1.0});
        }
        if (tr_radius <= std::numeric_limits<Scalar>::epsilon()) {
          break;
        }
      }
      return dir;
    }

    Scalar signed_gap(
      Vector3 const& point_first, Vector3 const& point_second,
      Vector3 const& normal) {
      Vector3 const difference = point_second - point_first;
      Scalar const distance = difference.norm();
      if (distance <= std::numeric_limits<Scalar>::epsilon()) {
        return 0.0;
      }
      return difference.dot(normal) >= 0.0 ? distance : -distance;
    }

    contact_feature_t point_point(
      collision_pair_t pair, geometry_ref_t const& first,
      geometry_ref_t const& second) {
      auto const& first_point =
        static_cast<PointGeometry const&>(first.geometry);
      auto const& second_point =
        static_cast<PointGeometry const&>(second.geometry);
      Vector3 const point_first = first_point.point(first.body.frameFromBody());
      Vector3 const point_second =
        second_point.point(second.body.frameFromBody());
      Vector3 const difference = point_second - point_first;
      Scalar const distance = difference.norm();
      Vector3 normal = Vector3::UnitZ();
      if (distance > std::numeric_limits<Scalar>::epsilon()) {
        normal = difference / distance;
      }
      return contact_feature_t {
        .pair = pair,
        .gap = distance,
        .point_first = point_first,
        .point_second = point_second,
        .normal = normal,
      };
    }

    contact_feature_t plane_point(
      collision_pair_t pair, geometry_ref_t const& plane_ref,
      geometry_ref_t const& point_ref) {
      auto const& plane = static_cast<PlaneGeometry const&>(plane_ref.geometry);
      auto const& point = static_cast<PointGeometry const&>(point_ref.geometry);
      Vector3 const point_second = point.point(point_ref.body.frameFromBody());
      Vector3 const normal = plane.normal(plane_ref.body.frameFromBody());
      Scalar const gap =
        plane.signedDistance(plane_ref.body.frameFromBody(), point_second);
      return contact_feature_t {
        .pair = pair,
        .gap = gap,
        .point_first = point_second - gap * normal,
        .point_second = point_second,
        .normal = normal,
      };
    }

    contact_feature_t plane_dsf(
      collision_pair_t pair, geometry_ref_t const& plane_ref,
      geometry_ref_t const& dsf_ref) {
      auto const& plane = static_cast<PlaneGeometry const&>(plane_ref.geometry);
      auto const& dsf = static_cast<DsfVertGeometry const&>(dsf_ref.geometry);
      Vector3 const normal = plane.normal(plane_ref.body.frameFromBody());
      support_t const sup = dsf.support(-normal, dsf_ref.body.frameFromBody());
      Scalar const gap =
        plane.signedDistance(plane_ref.body.frameFromBody(), sup.s);
      return contact_feature_t {
        .pair = pair,
        .gap = gap,
        .point_first = sup.s - gap * normal,
        .point_second = sup.s,
        .normal = normal,
      };
    }

    contact_feature_t dsf_point(
      collision_pair_t pair, geometry_ref_t const& dsf_ref,
      geometry_ref_t const& point_ref, narrow_phase_config_t const& config) {
      auto const& dsf = static_cast<DsfVertGeometry const&>(dsf_ref.geometry);
      auto const& point = static_cast<PointGeometry const&>(point_ref.geometry);
      Vector3 const point_second = point.point(point_ref.body.frameFromBody());
      Vector3 const dir = minimize_dir(
        point_second - geometry_center(dsf_ref), config.gain_ratio_lower_thresh,
        config, [&](Vector3 const& dir_eval) {
          diffable_support_t sup =
            dsf.diffable_support(dir_eval, dsf_ref.body.frameFromBody());
          sup.h -= dir_eval.dot(point_second);
          sup.s -= point_second;
          return sup;
        });
      diffable_support_t const sup =
        dsf.diffable_support(dir, dsf_ref.body.frameFromBody());
      return contact_feature_t {
        .pair = pair,
        .gap = signed_gap(sup.s, point_second, dir),
        .point_first = sup.s,
        .point_second = point_second,
        .normal = dir,
      };
    }

    contact_feature_t dsf_dsf(
      collision_pair_t pair, geometry_ref_t const& first,
      geometry_ref_t const& second, narrow_phase_config_t const& config) {
      auto const& first_dsf =
        static_cast<DsfVertGeometry const&>(first.geometry);
      auto const& second_dsf =
        static_cast<DsfVertGeometry const&>(second.geometry);
      Vector3 const dir = minimize_dir(
        geometry_center(second) - geometry_center(first),
        config.gain_ratio_lower_thresh / 2.0, config,
        [&](Vector3 const& dir_eval) {
          diffable_support_t const sup_1 =
            first_dsf.diffable_support(dir_eval, first.body.frameFromBody());
          diffable_support_t const sup_2 =
            second_dsf.diffable_support(-dir_eval, second.body.frameFromBody());
          diffable_support_t sup;
          sup.h = sup_1.h + sup_2.h;
          sup.s = sup_1.s - sup_2.s;
          sup.ds_dx = sup_1.ds_dx + sup_2.ds_dx;
          return sup;
        });
      diffable_support_t const sup_1 =
        first_dsf.diffable_support(dir, first.body.frameFromBody());
      diffable_support_t const sup_2 =
        second_dsf.diffable_support(-dir, second.body.frameFromBody());
      return contact_feature_t {
        .pair = pair,
        .gap = signed_gap(sup_1.s, sup_2.s, dir),
        .point_first = sup_1.s,
        .point_second = sup_2.s,
        .normal = dir,
      };
    }

    contact_feature_t reversed(
      contact_feature_t feature, collision_pair_t pair) {
      std::swap(feature.point_first, feature.point_second);
      feature.normal = -feature.normal;
      feature.pair = pair;
      return feature;
    }

    diffable_contact_feature_t diffable_plane_dsf(
      collision_pair_t pair, geometry_ref_t const& plane_ref,
      geometry_ref_t const& dsf_ref) {
      auto const& plane = static_cast<PlaneGeometry const&>(plane_ref.geometry);
      auto const& dsf = static_cast<DsfVertGeometry const&>(dsf_ref.geometry);
      pose_t const& frame_from_plane_body = plane_ref.body.frameFromBody();
      pose_t const& body_from_plane = plane.bodyFromGeometry();
      Vector3 const center = plane.center(frame_from_plane_body);
      Vector3 const normal = plane.normal(frame_from_plane_body);
      diffable_support_t const sup =
        dsf.diffable_support(-normal, dsf_ref.body.frameFromBody());
      Scalar const gap = normal.dot(sup.s - center);

      Matrix3 const R_fb = frame_from_plane_body.orientation.toRotationMatrix();
      Vector3 const normal_body =
        body_from_plane.orientation * Vector3::UnitZ();
      Matrix36 d_center = Matrix36::Zero();
      d_center.leftCols<3>() = Matrix3::Identity();
      d_center.rightCols<3>() = -R_fb * skew(body_from_plane.position);
      Matrix36 d_normal = Matrix36::Zero();
      d_normal.rightCols<3>() = -R_fb * skew(normal_body);

      diffable_contact_feature_t out;
      out.pair = pair;
      out.gap = gap;
      out.point_second = sup.s;
      out.point_first = sup.s - gap * normal;
      out.normal = normal;
      out.d_normal.leftCols<6>() = d_normal;
      out.d_point_second.leftCols<6>() = -sup.ds_dx * d_normal;
      out.d_point_second.rightCols<6>() = sup.ds_dq;
      out.d_gap.leftCols<6>() =
        (sup.s - center).transpose() * d_normal - normal.transpose() * d_center;
      out.d_gap.rightCols<6>() = normal.transpose() * sup.ds_dq;
      out.d_point_first =
        out.d_point_second - normal * out.d_gap - gap * out.d_normal;
      return out;
    }

    diffable_contact_feature_t reversed(
      diffable_contact_feature_t feature, collision_pair_t pair) {
      std::swap(feature.point_first, feature.point_second);
      Eigen::Matrix<Scalar, 3, 12> const d_point_first = feature.d_point_first;
      Eigen::Matrix<Scalar, 3, 12> const d_point_second =
        feature.d_point_second;
      feature.d_point_first.leftCols<6>() = d_point_second.rightCols<6>();
      feature.d_point_first.rightCols<6>() = d_point_second.leftCols<6>();
      feature.d_point_second.leftCols<6>() = d_point_first.rightCols<6>();
      feature.d_point_second.rightCols<6>() = d_point_first.leftCols<6>();
      Eigen::Matrix<Scalar, 1, 6> const d_gap_first =
        feature.d_gap.leftCols<6>();
      feature.d_gap.leftCols<6>() = feature.d_gap.rightCols<6>();
      feature.d_gap.rightCols<6>() = d_gap_first;
      Eigen::Matrix<Scalar, 3, 6> const d_normal_first =
        feature.d_normal.leftCols<6>();
      feature.d_normal.leftCols<6>() = -feature.d_normal.rightCols<6>();
      feature.d_normal.rightCols<6>() = -d_normal_first;
      feature.normal = -feature.normal;
      feature.pair = pair;
      return feature;
    }

  }  // namespace

  contact_feature_t compute_contact(
    SceneSnapshot const& scene, collision_pair_t pair,
    narrow_phase_config_t const& config) {
    if (pair.first.entity == pair.second.entity) {
      throw std::invalid_argument(
        "collision pair geometries must belong to distinct entities");
    }
    geometry_ref_t const first = find_geometry(scene, pair.first);
    geometry_ref_t const second = find_geometry(scene, pair.second);
    geometry_type_e const first_type = first.geometry.type();
    geometry_type_e const second_type = second.geometry.type();
    if (!supports_collision_pair(first_type, second_type)) {
      throw std::invalid_argument("collision geometry pair is not supported");
    }

    if (
      first_type == geometry_type_e::point &&
      second_type == geometry_type_e::point) {
      return point_point(pair, first, second);
    }
    if (
      first_type == geometry_type_e::plane &&
      second_type == geometry_type_e::point) {
      return plane_point(pair, first, second);
    }
    if (
      first_type == geometry_type_e::point &&
      second_type == geometry_type_e::plane) {
      return reversed(plane_point(pair, second, first), pair);
    }
    if (
      first_type == geometry_type_e::plane &&
      second_type == geometry_type_e::dsf_vert) {
      return plane_dsf(pair, first, second);
    }
    if (
      first_type == geometry_type_e::dsf_vert &&
      second_type == geometry_type_e::plane) {
      return reversed(plane_dsf(pair, second, first), pair);
    }
    if (
      first_type == geometry_type_e::dsf_vert &&
      second_type == geometry_type_e::point) {
      return dsf_point(pair, first, second, config);
    }
    if (
      first_type == geometry_type_e::point &&
      second_type == geometry_type_e::dsf_vert) {
      return reversed(dsf_point(pair, second, first, config), pair);
    }
    if (
      first_type == geometry_type_e::dsf_vert &&
      second_type == geometry_type_e::dsf_vert) {
      return dsf_dsf(pair, first, second, config);
    }
    throw std::logic_error("unhandled collision geometry pair");
  }

  diffable_contact_feature_t compute_diffable_contact(
    SceneSnapshot const& scene, collision_pair_t pair,
    narrow_phase_config_t const& config) {
    geometry_ref_t const first = find_geometry(scene, pair.first);
    geometry_ref_t const second = find_geometry(scene, pair.second);
    if (
      first.geometry.type() == geometry_type_e::plane &&
      second.geometry.type() == geometry_type_e::dsf_vert) {
      return diffable_plane_dsf(pair, first, second);
    }
    if (
      first.geometry.type() == geometry_type_e::dsf_vert &&
      second.geometry.type() == geometry_type_e::plane) {
      return reversed(diffable_plane_dsf(pair, second, first), pair);
    }
    if (
      first.geometry.type() != geometry_type_e::dsf_vert ||
      second.geometry.type() != geometry_type_e::dsf_vert) {
      throw std::invalid_argument(
        "diffable contact requires DSF-DSF or plane-DSF geometries");
    }

    auto const& dsf_1 = static_cast<DsfVertGeometry const&>(first.geometry);
    auto const& dsf_2 = static_cast<DsfVertGeometry const&>(second.geometry);
    Vector3 const dir = minimize_dir(
      geometry_center(second) - geometry_center(first),
      config.gain_ratio_lower_thresh / 2.0, config,
      [&](Vector3 const& dir_eval) {
        diffable_support_t const sup_1 =
          dsf_1.diffable_support(dir_eval, first.body.frameFromBody());
        diffable_support_t const sup_2 =
          dsf_2.diffable_support(-dir_eval, second.body.frameFromBody());
        diffable_support_t sup;
        sup.h = sup_1.h + sup_2.h;
        sup.s = sup_1.s - sup_2.s;
        sup.ds_dx = sup_1.ds_dx + sup_2.ds_dx;
        return sup;
      });

    diffable_support_t const sup_1 =
      dsf_1.diffable_support(dir, first.body.frameFromBody());
    diffable_support_t const sup_2 =
      dsf_2.diffable_support(-dir, second.body.frameFromBody());
    Vector3 const point_delta = sup_2.s - sup_1.s;
    Matrix3 const P = Matrix3::Identity() - dir * dir.transpose();
    Eigen::Matrix<Scalar, 3, 12> support_pose;
    support_pose << sup_1.ds_dq, -sup_2.ds_dq;
    Eigen::Matrix<Scalar, 3, 12> const d_dir =
      (dir * (sup_1.s - sup_2.s).transpose() +
       dir.dot(sup_1.s - sup_2.s) * Matrix3::Identity() -
       P * (sup_1.ds_dx + sup_2.ds_dx))
        .inverse() *
      P * support_pose;

    diffable_contact_feature_t out;
    out.pair = pair;
    out.point_first = sup_1.s;
    out.point_second = sup_2.s;
    out.normal = dir;
    out.d_point_first = sup_1.ds_dx * d_dir;
    out.d_point_first.leftCols<6>() += sup_1.ds_dq;
    out.d_point_second = -sup_2.ds_dx * d_dir;
    out.d_point_second.rightCols<6>() += sup_2.ds_dq;
    Scalar const sign = point_delta.dot(dir) >= 0.0 ? 1.0 : -1.0;
    out.gap = sign * point_delta.norm();
    if (point_delta.norm() > std::numeric_limits<Scalar>::epsilon()) {
      out.d_gap = sign * point_delta.normalized().transpose() *
        (out.d_point_second - out.d_point_first);
    }
    out.d_normal = P * d_dir;
    return out;
  }

}  // namespace stacking_core
