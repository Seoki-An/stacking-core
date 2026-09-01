#include "force.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace stacking_core::grasp_detail {
  namespace {

    struct cone_projection_t {
      Vector3 force = Vector3::Zero();
      Matrix3 d_force_d_normal = Matrix3::Zero();
      Matrix3 d_force_d_force = Matrix3::Zero();
    };

    cone_projection_t project_friction_cone(
      Vector3 const& force, Vector3 const& normal, Scalar friction) {
      cone_projection_t out;
      Scalar const norm = force.norm();
      if (norm < 1e-6) {
        return out;
      }

      Scalar const cosine = std::clamp(normal.dot(force / norm), -1.0, 1.0);
      Scalar const angle = std::acos(cosine);
      Scalar const cone_angle = std::atan(friction);
      if (angle >= cone_angle + std::numbers::pi_v<Scalar> / 2.0) {
        return out;
      }
      if (angle <= cone_angle) {
        out.force = force;
        out.d_force_d_force = Matrix3::Identity();
        return out;
      }

      Vector3 const tangent = force - force.dot(normal) * normal;
      Vector3 const u = tangent.normalized();
      Vector3 const v = normal + friction * u;
      Vector3 const w = v.normalized();
      out.force = force.dot(w) * w;

      Matrix3 const d_force_d_w =
        w * force.transpose() + force.dot(w) * Matrix3::Identity();
      Matrix3 const d_w_d_v =
        (Matrix3::Identity() - w * w.transpose()) / v.norm();
      Matrix3 const d_v_d_u = friction * Matrix3::Identity();
      Matrix3 const d_u_d_tangent =
        (Matrix3::Identity() - u * u.transpose()) / tangent.norm();
      Matrix3 const d_tangent_d_normal =
        -force.dot(normal) * Matrix3::Identity() - normal * force.transpose();
      Matrix3 const d_v_d_normal =
        Matrix3::Identity() + d_v_d_u * d_u_d_tangent * d_tangent_d_normal;
      Matrix3 const d_tangent_d_force =
        Matrix3::Identity() - normal * normal.transpose();

      out.d_force_d_normal = d_force_d_w * d_w_d_v * d_v_d_normal;
      out.d_force_d_force = w * w.transpose() +
        d_force_d_w * d_w_d_v * d_v_d_u * d_u_d_tangent * d_tangent_d_force;
      return out;
    }

    std::array<Vector6, 5> disturbance_wrenches(Scalar scale) {
      std::array<Vector6, 5> out;
      out[0] << 0.0, 0.0, -10.0, 0.0, 0.0, 0.0;
      out[1] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      out[2] << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      out[3] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
      out[4] << 0.0, -1.0, 0.0, 0.0, 0.0, 0.0;
      for (Vector6& wrench : out) {
        wrench *= scale;
      }
      return out;
    }

    Eigen::Matrix<Scalar, 6, 3> wrench_map(
      Vector3 const& point, Vector3 const& center) {
      Eigen::Matrix<Scalar, 6, 3> G;
      G.topRows<3>() = Matrix3::Identity();
      G.bottomRows<3>() = skew(point - center);
      return G;
    }

  }  // namespace

  bool force_evaluation_t::finite() const noexcept {
    return std::isfinite(cost) && grad.allFinite() && hess.allFinite();
  }

  bool force_refinement_t::finite() const noexcept {
    if (!std::isfinite(cost) || !grad.allFinite()) {
      return false;
    }
    return std::all_of(
      contact_forces.begin(), contact_forces.end(),
      [](Vector3 const& force) { return force.allFinite(); });
  }

  force_evaluation_t evaluate_contact_forces(
    Eigen::Ref<Eigen::VectorXd const> forces,
    std::vector<force_contact_t> const& contacts, Vector6 const& wrench,
    Vector3 const& center, grasp_force_config_t const& config) {
    Eigen::Index const count = static_cast<Eigen::Index>(contacts.size());
    force_evaluation_t out;
    out.grad = Eigen::VectorXd::Zero(3 * count);
    out.hess = Eigen::MatrixXd::Zero(3 * count, 3 * count);

    Matrix6 W = Matrix6::Identity();
    W.bottomRightCorner<3, 3>() *= config.weight.moment;
    Vector6 total = Vector6::Zero();
    std::vector<Eigen::Matrix<Scalar, 6, 3>> maps;
    maps.reserve(contacts.size());
    for (Eigen::Index i = 0; i < count; ++i) {
      auto const& contact = contacts[static_cast<std::size_t>(i)];
      auto const G = wrench_map(contact.feature.point_second, center);
      maps.push_back(G);
      total += G * forces.segment<3>(3 * i);
    }
    Vector6 const residual = wrench + total;
    out.cost += config.weight.wrench * 0.5 * residual.dot(W * residual);
    for (Eigen::Index i = 0; i < count; ++i) {
      auto const& G_i = maps[static_cast<std::size_t>(i)];
      out.grad.segment<3>(3 * i) +=
        config.weight.wrench * G_i.transpose() * W * residual;
      for (Eigen::Index j = 0; j < count; ++j) {
        auto const& G_j = maps[static_cast<std::size_t>(j)];
        out.hess.block<3, 3>(3 * i, 3 * j) +=
          config.weight.wrench * G_i.transpose() * W * G_j;
      }
    }

    for (Eigen::Index i = 0; i < count; ++i) {
      force_contact_t const& contact = contacts[static_cast<std::size_t>(i)];
      Vector3 const force = forces.segment<3>(3 * i);
      Scalar const gap = contact.feature.gap;
      out.cost +=
        config.weight.complementarity * 0.5 * gap * gap * force.squaredNorm();
      out.grad.segment<3>(3 * i) +=
        config.weight.complementarity * gap * gap * force;
      out.hess.block<3, 3>(3 * i, 3 * i) +=
        config.weight.complementarity * gap * gap * Matrix3::Identity();

      cone_projection_t const projection =
        project_friction_cone(force, contact.feature.normal, config.friction);
      Vector3 const cone_residual = force - projection.force;
      out.cost += config.weight.cone * 0.5 * cone_residual.squaredNorm();
      out.grad.segment<3>(3 * i) += config.weight.cone * cone_residual;
      out.hess.block<3, 3>(3 * i, 3 * i) +=
        config.weight.cone * (Matrix3::Identity() - projection.d_force_d_force);
    }
    return out;
  }

  force_refinement_t refine_contact_forces(
    std::vector<force_contact_t> const& contacts, Vector3 const& center,
    grasp_force_config_t const& config) {
    constexpr int max_iters = 100;
    constexpr int max_interval_iters = 50;
    constexpr int max_line_iters = 50;
    constexpr Scalar grad_tol = 1e-12;
    constexpr Scalar line_tol = 1e-6;

    force_refinement_t out;
    Eigen::Index const count = static_cast<Eigen::Index>(contacts.size());
    Eigen::VectorXd const initial = Eigen::VectorXd::Zero(3 * count);
    for (Vector6 const& wrench : disturbance_wrenches(config.wrench_scale)) {
      Eigen::VectorXd forces = initial;
      for (int iter = 0; iter < max_iters; ++iter) {
        force_evaluation_t const eval =
          evaluate_contact_forces(forces, contacts, wrench, center, config);
        if (eval.grad.norm() < grad_tol) {
          break;
        }

        Eigen::MatrixXd H = 0.5 * (eval.hess + eval.hess.transpose());
        H.diagonal().array() += config.damping;
        Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
        Eigen::VectorXd step;
        if (ldlt.info() == Eigen::Success) {
          step = ldlt.solve(-eval.grad);
        } else {
          step = -eval.grad;
        }
        if (!step.allFinite()) {
          step = -eval.grad;
        }

        Scalar low = 0.0;
        Scalar high = 1.0;
        bool found_interval = false;
        for (int interval_iter = 0; interval_iter < max_interval_iters;
             ++interval_iter) {
          force_evaluation_t const trial = evaluate_contact_forces(
            forces + high * step, contacts, wrench, center, config);
          if (step.dot(trial.grad) > 0.0) {
            found_interval = true;
            break;
          }
          high *= 2.0;
        }
        if (!found_interval) {
          break;
        }

        Scalar alpha = 0.5 * (low + high);
        for (int line_iter = 0; line_iter < max_line_iters; ++line_iter) {
          Eigen::VectorXd const trial_forces = forces + alpha * step;
          force_evaluation_t const trial = evaluate_contact_forces(
            trial_forces, contacts, wrench, center, config);
          Scalar const dir_grad = step.dot(trial.grad);
          if (std::abs(dir_grad) < line_tol) {
            forces = trial_forces;
            break;
          }
          if (dir_grad > 0.0) {
            high = alpha;
          } else {
            low = alpha;
          }
          Scalar const dir_hess = step.dot(trial.hess * step);
          Scalar const candidate = alpha - dir_grad / dir_hess;
          alpha = candidate > low && candidate < high ? candidate
                                                      : 0.5 * (low + high);
          if (alpha == 0.0) {
            break;
          }
        }
      }

      Matrix6 W = Matrix6::Identity();
      W.bottomRightCorner<3, 3>() *= config.weight.moment;
      Vector6 total = Vector6::Zero();
      for (Eigen::Index i = 0; i < count; ++i) {
        auto const& contact = contacts[static_cast<std::size_t>(i)];
        total += wrench_map(contact.feature.point_second, center) *
          forces.segment<3>(3 * i);
      }
      Vector6 const residual = wrench + total;
      out.cost += config.weight.wrench * 0.5 * residual.dot(W * residual);
      for (Eigen::Index i = 0; i < count; ++i) {
        force_contact_t const& contact = contacts[static_cast<std::size_t>(i)];
        Vector3 const force = forces.segment<3>(3 * i);
        matrix67_t S = matrix67_t::Zero();
        S.topRows<3>() = skew(force) *
          contact.feature.d_point_second.leftCols<6>() * contact.body_jac;
        out.grad += config.weight.wrench * S.transpose() * W * residual;

        Scalar const gap = contact.feature.gap;
        out.cost +=
          config.weight.complementarity * 0.5 * gap * gap * force.squaredNorm();
        out.grad += config.weight.complementarity * gap * force.squaredNorm() *
          (contact.feature.d_gap.leftCols<6>() * contact.body_jac).transpose();

        cone_projection_t const projection =
          project_friction_cone(force, contact.feature.normal, config.friction);
        Vector3 const cone_residual = force - projection.force;
        out.cost += config.weight.cone * 0.5 * cone_residual.squaredNorm();
        out.grad += config.weight.cone *
          -(projection.d_force_d_normal *
            contact.feature.d_normal.leftCols<6>() * contact.body_jac)
             .transpose() *
          cone_residual;
      }

      out.contact_forces.resize(contacts.size());
      for (Eigen::Index i = 0; i < count; ++i) {
        out.contact_forces[static_cast<std::size_t>(i)] =
          forces.segment<3>(3 * i);
      }
    }
    return out;
  }

}  // namespace stacking_core::grasp_detail
