#include <stacking_core/planner/inverse_kinematics.hpp>

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

namespace stacking_core {
  namespace {

    constexpr Scalar k_p = 1.0;
    constexpr Scalar k_omega = 2.0;
    constexpr Scalar joint_limit_weight = 1000.0;
    constexpr Scalar joint_margin = 0.01;
    constexpr Scalar lambda_init = 1e-2;
    constexpr Scalar lambda_min = 1e-8;
    constexpr Scalar lambda_max = 1e6;
    constexpr Scalar lambda_grow = 4.0;
    constexpr Scalar lambda_shrink = 0.5;
    constexpr int stall_limit = 50;
    constexpr Scalar stall_min_improve = 1e-12;

    struct joint_bounds_t {
      Eigen::VectorXd lower;
      Eigen::VectorXd upper;
      std::vector<bool> bounded;
      bool valid = true;
    };

    struct evaluation_t {
      pose_t frame_from_link;
      Matrix6X jac;
      Vector6 error = Vector6::Zero();
      Scalar pos_error_sq = 0.0;
      Scalar rot_error = 0.0;
    };

    struct limit_constraint_t {
      Eigen::Index dof = 0;
      Scalar multiplier = 1.0;
      Scalar offset = 0.0;
      Scalar lower = 0.0;
      Scalar upper = 0.0;
    };

    struct limit_constraints_t {
      std::vector<limit_constraint_t> values;
      bool valid = true;
    };

    struct limit_terms_t {
      Eigen::VectorXd grad;
      Eigen::VectorXd hess_diag;
      Scalar max_violation = 0.0;
    };

    inverse_kinematics_result_t invalid_result(
      inverse_kinematics_problem_t const& problem, std::string code,
      std::string message) {
      return inverse_kinematics_result_t {
        .status = solve_status_e::invalid_problem,
        .positions = problem.initial_state.positions(),
        .frame_from_link = {},
        .pos_error = std::numeric_limits<Scalar>::infinity(),
        .rot_error = std::numeric_limits<Scalar>::infinity(),
        .solver = {},
        .failure =
          {
            .code = std::move(code),
            .message = std::move(message),
            .retryable = false,
          },
      };
    }

    LinkId root_of(KinematicModel const& model, LinkId link) {
      while (kinematic_joint_t const* joint = model.parentJoint(link)) {
        link = joint->parent;
      }
      return link;
    }

    std::optional<std::size_t> proximal_dof(
      KinematicModel const& model, LinkId link) {
      std::optional<std::size_t> result;
      while (kinematic_joint_t const* joint = model.parentJoint(link)) {
        std::optional<std::size_t> const index =
          model.degreeOfFreedomIndex(joint->id);
        if (index.has_value()) {
          result = index;
        }
        link = joint->parent;
      }
      return result;
    }

    joint_bounds_t make_joint_bounds(KinematicModel const& model) {
      Eigen::Index const size =
        static_cast<Eigen::Index>(model.degreeOfFreedomCount());
      joint_bounds_t bounds {
        .lower = Eigen::VectorXd::Zero(size),
        .upper = Eigen::VectorXd::Zero(size),
        .bounded = std::vector<bool>(static_cast<std::size_t>(size), false),
        .valid = true,
      };
      for (Eigen::Index i = 0; i < size; ++i) {
        kinematic_joint_t const& joint =
          model.joint(model.degreeOfFreedomJoint(static_cast<std::size_t>(i)));
        if (!joint.limit.has_value()) {
          continue;
        }
        bounds.lower(i) = joint.limit->lower + joint_margin;
        bounds.upper(i) = joint.limit->upper - joint_margin;
        bounds.bounded[static_cast<std::size_t>(i)] = true;
        bounds.valid = bounds.valid && bounds.lower(i) <= bounds.upper(i);
      }
      return bounds;
    }

    limit_constraints_t make_limit_constraints(KinematicModel const& model) {
      Eigen::Index const size =
        static_cast<Eigen::Index>(model.degreeOfFreedomCount());
      Eigen::VectorXd const zero = Eigen::VectorXd::Zero(size);
      limit_constraints_t constraints;
      for (std::size_t i = 0; i < model.jointCount(); ++i) {
        kinematic_joint_t const& joint = model.joint(i);
        if (!joint.limit.has_value()) {
          continue;
        }
        std::optional<std::size_t> const dof =
          model.degreeOfFreedomIndex(joint.id);
        if (!dof.has_value()) {
          constraints.valid = false;
          continue;
        }

        Scalar const offset = model.jointPosition(joint.id, zero);
        Eigen::VectorXd unit = zero;
        unit(static_cast<Eigen::Index>(*dof)) = 1.0;
        Scalar const multiplier = model.jointPosition(joint.id, unit) - offset;
        Scalar const lower = joint.limit->lower + joint_margin;
        Scalar const upper = joint.limit->upper - joint_margin;
        constraints.values.push_back(limit_constraint_t {
          .dof = static_cast<Eigen::Index>(*dof),
          .multiplier = multiplier,
          .offset = offset,
          .lower = lower,
          .upper = upper,
        });
        constraints.valid = constraints.valid && lower <= upper;
      }
      return constraints;
    }

    Scalar wrap_to_pi(Scalar angle) {
      angle = std::fmod(angle + std::numbers::pi, 2.0 * std::numbers::pi);
      if (angle < 0.0) {
        angle += 2.0 * std::numbers::pi;
      }
      return angle - std::numbers::pi;
    }

    Eigen::VectorXd wrap_revolute_positions(
      KinematicModel const& model, Eigen::VectorXd positions) {
      for (std::size_t i = 0; i < model.degreeOfFreedomCount(); ++i) {
        kinematic_joint_t const& joint =
          model.joint(model.degreeOfFreedomJoint(i));
        if (joint.type == joint_type_e::revolute) {
          positions(static_cast<Eigen::Index>(i)) =
            wrap_to_pi(positions(static_cast<Eigen::Index>(i)));
        }
      }
      return positions;
    }

    void initialize_swing_toward_target(
      KinematicState& state, LinkId link, pose_t const& target,
      joint_bounds_t const& bounds) {
      KinematicModel const& model = state.model();
      std::optional<std::size_t> const dof = proximal_dof(model, link);
      if (!dof.has_value() || !bounds.bounded[*dof]) {
        return;
      }

      LinkId const root = root_of(model, link);
      pose_t const root_from_frame = inverse(state.frameFromRoot(root));
      Vector3 const target_root =
        transform_point(root_from_frame, target.position);
      if (target_root.head<2>().norm() < 1e-9) {
        return;
      }

      auto azimuth_at = [&](Eigen::VectorXd const& positions, Scalar& azimuth) {
        state.setPositions(positions);
        KinematicSnapshot const snapshot = forward_kinematics(state);
        pose_t const& frame_from_link = snapshot.frameFromLink(link);
        Vector3 const link_root =
          transform_point(root_from_frame, frame_from_link.position);
        if (link_root.head<2>().norm() < 1e-9) {
          return false;
        }
        azimuth = std::atan2(link_root.y(), link_root.x());
        return true;
      };

      Eigen::Index const index = static_cast<Eigen::Index>(*dof);
      Eigen::VectorXd positions = state.positions();
      Scalar azimuth = 0.0;
      if (!azimuth_at(positions, azimuth)) {
        return;
      }

      constexpr Scalar probe_step = 1e-3;
      Scalar step = probe_step;
      if (positions(index) + step > bounds.upper(index)) {
        step = -probe_step;
      }
      if (positions(index) + step < bounds.lower(index)) {
        return;
      }
      Eigen::VectorXd probe = positions;
      probe(index) += step;
      Scalar probe_azimuth = 0.0;
      if (!azimuth_at(probe, probe_azimuth)) {
        return;
      }
      Scalar const azimuth_step =
        std::remainder(probe_azimuth - azimuth, 2.0 * std::numbers::pi);
      Scalar const azimuth_per_joint = azimuth_step / step;
      if (std::abs(azimuth_per_joint) < 1e-6) {
        return;
      }

      Scalar const target_azimuth =
        std::atan2(target_root.y(), target_root.x());
      Scalar const azimuth_error =
        std::remainder(target_azimuth - azimuth, 2.0 * std::numbers::pi);
      positions(index) = std::clamp(
        positions(index) + azimuth_error / azimuth_per_joint,
        bounds.lower(index), bounds.upper(index));
      state.setPositions(positions);
    }

    evaluation_t evaluate(
      KinematicState& state, LinkId link, pose_t const& target,
      bool position_only, Eigen::VectorXd const& positions) {
      state.setPositions(positions);
      KinematicSnapshot const snapshot = forward_kinematics(state);
      pose_t const& achieved = snapshot.frameFromLink(link);
      Matrix6X jac = snapshot.linkJacobian(link);

      // The legacy planner used a hybrid Jacobian: scene-frame translation and
      // end-link-frame rotation. Preserve that convention for numerical parity.
      Matrix3 const R = achieved.orientation.toRotationMatrix();
      jac.bottomRows<3>() = R.transpose() * jac.bottomRows<3>();

      evaluation_t result {
        .frame_from_link = achieved,
        .jac = std::move(jac),
        .error = Vector6::Zero(),
        .pos_error_sq = 0.0,
        .rot_error = 0.0,
      };
      Vector3 const pos_error = achieved.position - target.position;
      result.error.head<3>() = k_p * pos_error;
      result.pos_error_sq = pos_error.squaredNorm();
      if (position_only) {
        result.jac.bottomRows<3>().setZero();
        return result;
      }

      Matrix3 const R_error =
        (target.orientation.conjugate() * achieved.orientation)
          .toRotationMatrix();
      result.error.tail<3>() = k_omega * 0.5 *
        Vector3 {
          -(R_error - R_error.transpose())(1, 2),
          (R_error - R_error.transpose())(0, 2),
          -(R_error - R_error.transpose())(0, 1),
        };
      result.rot_error = 0.5 * (Matrix3::Identity() - R_error).trace();
      return result;
    }

    limit_terms_t evaluate_limits(
      Eigen::VectorXd const& positions,
      limit_constraints_t const& constraints) {
      limit_terms_t terms {
        .grad = Eigen::VectorXd::Zero(positions.size()),
        .hess_diag = Eigen::VectorXd::Zero(positions.size()),
        .max_violation = 0.0,
      };
      for (limit_constraint_t const& constraint : constraints.values) {
        Scalar const value =
          constraint.multiplier * positions(constraint.dof) + constraint.offset;
        Scalar const upper = std::max(value - constraint.upper, Scalar {0.0});
        Scalar const lower = std::max(constraint.lower - value, Scalar {0.0});
        Scalar const active_count =
          (upper > 0.0 ? 1.0 : 0.0) + (lower > 0.0 ? 1.0 : 0.0);
        terms.grad(constraint.dof) +=
          joint_limit_weight * constraint.multiplier * (upper - lower);
        terms.hess_diag(constraint.dof) += joint_limit_weight *
          constraint.multiplier * constraint.multiplier * active_count;
        terms.max_violation =
          std::max(terms.max_violation, std::max(upper, lower));
      }
      return terms;
    }

  }  // namespace

  inverse_kinematics_result_t solve_inverse_kinematics(
    inverse_kinematics_problem_t const& problem,
    inverse_kinematics_config_t const& config) {
    KinematicModel const& model = problem.initial_state.model();
    if (model.findLink(problem.link) == nullptr) {
      return invalid_result(
        problem, "unknown_link", "IK link is not in the model");
    }
    if (!is_valid(problem.frame_from_link)) {
      return invalid_result(
        problem, "invalid_target",
        "IK target pose must be finite and normalized");
    }
    if (
      config.max_iters <= 0 || !std::isfinite(config.tol) ||
      config.tol <= 0.0) {
      return invalid_result(
        problem, "invalid_config",
        "IK iterations and tolerance must be positive");
    }

    joint_bounds_t const bounds = make_joint_bounds(model);
    limit_constraints_t const limit_constraints = make_limit_constraints(model);
    if (!bounds.valid || !limit_constraints.valid) {
      return invalid_result(
        problem, "narrow_joint_limit",
        "IK joint range is narrower than the legacy joint margin");
    }

    KinematicState state = problem.initial_state;
    if (config.initialization == inverse_kinematics_initialization_e::swing) {
      initialize_swing_toward_target(
        state, problem.link, problem.frame_from_link, bounds);
    }
    Eigen::VectorXd positions = state.positions();
    evaluation_t current = evaluate(
      state, problem.link, problem.frame_from_link, problem.position_only,
      positions);

    Scalar lambda = lambda_init;
    Scalar best_residual = std::numeric_limits<Scalar>::infinity();
    Scalar grad_norm = 0.0;
    int stall_count = 0;
    int iters = 0;
    bool converged = false;
    bool numerical_failure = false;
    bool stalled = false;
    Eigen::Index const n = positions.size();

    for (int iter = 0; iter < config.max_iters; ++iter) {
      limit_terms_t const limits =
        evaluate_limits(positions, limit_constraints);
      Scalar const pose_residual = current.pos_error_sq + current.rot_error;
      if (pose_residual < config.tol && limits.max_violation < config.tol) {
        converged = true;
        break;
      }
      if (pose_residual < best_residual - stall_min_improve) {
        best_residual = pose_residual;
        stall_count = 0;
      } else if (++stall_count >= stall_limit) {
        stalled = true;
        break;
      }
      if (n == 0) {
        stalled = true;
        break;
      }

      Eigen::MatrixXd A = current.jac.transpose() * current.jac +
        lambda * Eigen::MatrixXd::Identity(n, n);
      for (Eigen::Index i = 0; i < n; ++i) {
        A(i, i) += limits.hess_diag(i);
      }
      Eigen::VectorXd const grad =
        current.jac.transpose() * current.error + limits.grad;
      grad_norm = grad.norm();
      Eigen::VectorXd const dq = A.llt().solve(grad);
      if (!dq.allFinite()) {
        numerical_failure = true;
        break;
      }

      evaluation_t trial = evaluate(
        state, problem.link, problem.frame_from_link, problem.position_only,
        positions - dq);
      Scalar const trial_residual = trial.pos_error_sq + trial.rot_error;
      if (trial_residual < pose_residual) {
        positions -= dq;
        current = std::move(trial);
        lambda = std::max(lambda_min, lambda * lambda_shrink);
      } else {
        lambda = std::min(lambda_max, lambda * lambda_grow);
      }
      iters = iter + 1;
    }

    positions = wrap_revolute_positions(model, std::move(positions));
    current = evaluate(
      state, problem.link, problem.frame_from_link, problem.position_only,
      positions);
    Scalar const objective = current.pos_error_sq + current.rot_error;

    solve_status_e status = solve_status_e::success;
    planner_failure_t failure;
    if (!converged) {
      status = iters == config.max_iters ? solve_status_e::max_iters
                                         : solve_status_e::infeasible;
      failure = planner_failure_t {
        .code = numerical_failure               ? "numerical_failure"
          : stalled                             ? "stalled"
          : status == solve_status_e::max_iters ? "max_iters"
                                                : "infeasible",
        .message = numerical_failure
          ? "IK linear solve produced a non-finite step"
          : stalled ? "IK residual stopped improving"
          : status == solve_status_e::max_iters
          ? "IK reached the iteration limit"
          : "IK did not satisfy its residual and joint limits",
        .retryable = true,
      };
    }

    return inverse_kinematics_result_t {
      .status = status,
      .positions = std::move(positions),
      .frame_from_link = std::move(current.frame_from_link),
      .pos_error = std::sqrt(current.pos_error_sq),
      .rot_error = current.rot_error,
      .solver =
        {
          .iters = iters,
          .converged = converged,
          .objective = objective,
          .grad_norm = grad_norm,
        },
      .failure = std::move(failure),
    };
  }

}  // namespace stacking_core
