#include "evaluation.hpp"

#include <stacking_core/optimization/truncated_conjugate_gradient.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace stacking_core {
  namespace {

    struct augmented_value_t {
      Scalar objective = 0.0;
      grasp_detail::vector7_t grad = grasp_detail::vector7_t::Zero();
    };

    augmented_value_t augmented_value(
      grasp_detail::grasp_evaluation_t const& eval,
      Eigen::VectorXd const& dual_ineq, Eigen::VectorXd const& dual_eq,
      Scalar beta_ineq, Scalar beta_eq) {
      Eigen::VectorXd const active =
        (dual_ineq + beta_ineq * eval.c_ineq).cwiseMax(0.0);
      return augmented_value_t {
        .objective = eval.objective +
          (active.squaredNorm() - dual_ineq.squaredNorm()) / (2.0 * beta_ineq) +
          dual_eq.dot(eval.c_eq) + 0.5 * beta_eq * eval.c_eq.squaredNorm(),
        .grad = eval.grad + eval.jac_ineq.transpose() * active +
          eval.jac_eq.transpose() * dual_eq +
          beta_eq * eval.jac_eq.transpose() * eval.c_eq,
      };
    }

    Scalar max_inequality(Eigen::VectorXd const& values) {
      return values.size() == 0 ? -std::numeric_limits<Scalar>::infinity()
                                : values.maxCoeff();
    }

    Scalar max_equality(Eigen::VectorXd const& values) {
      return values.size() == 0 ? 0.0 : values.cwiseAbs().maxCoeff();
    }

    bool feasible(
      grasp_detail::grasp_evaluation_t const& eval,
      grasp_alm_config_t const& config) {
      return eval.finite() &&
        max_inequality(eval.c_ineq) <= config.inequality_tol &&
        max_equality(eval.c_eq) <= config.equality_tol;
    }

    planner_failure_t solve_failure(
      grasp_detail::grasp_evaluation_t const& eval,
      grasp_alm_config_t const& config) {
      if (!eval.finite()) {
        return planner_failure_t {
          .code = "nonfinite_grasp_evaluation",
          .message = "grasp objective or constraints became non-finite",
          .retryable = true,
        };
      }
      return planner_failure_t {
        .code = "grasp_constraints_infeasible",
        .message = "grasp inequality violation " +
          std::to_string(max_inequality(eval.c_ineq)) + " exceeds tolerance " +
          std::to_string(config.inequality_tol),
        .retryable = true,
      };
    }

  }  // namespace

  grasp_result_t solve_grasp_pose(
    grasp_problem_t const& problem, grasp_generation_config_t const& config) {
    if (
      auto const failure =
        grasp_detail::validate_grasp_problem(problem, config)) {
      return grasp_result_t {
        .status = solve_status_e::invalid_problem,
        .candidates = {},
        .selected_index = std::nullopt,
        .failure = *failure,
      };
    }

    grasp_t grasp = problem.seed;
    grasp_detail::grasp_evaluation_t eval =
      grasp_detail::evaluate_grasp(problem, config, grasp);
    Eigen::VectorXd dual_ineq = Eigen::VectorXd::Zero(eval.c_ineq.size());
    Eigen::VectorXd dual_eq = Eigen::VectorXd::Zero(eval.c_eq.size());
    Scalar beta_ineq = config.alm.inequality_penalty_init;
    Scalar beta_eq = config.alm.equality_penalty_init;
    Scalar prev_ineq = std::numeric_limits<Scalar>::infinity();
    Scalar prev_eq = std::numeric_limits<Scalar>::infinity();
    int total_iters = 0;

    for (int alm_iter = 0; alm_iter < config.alm.max_iters; ++alm_iter) {
      Scalar radius = config.trust_region.radius_init;
      grasp_detail::matrix7_t hess = grasp_detail::matrix7_t::Identity();
      for (int iter = 0; iter < config.trust_region.max_iters; ++iter) {
        ++total_iters;
        augmented_value_t const current =
          augmented_value(eval, dual_ineq, dual_eq, beta_ineq, beta_eq);
        auto const [step, boundary] = truncated_conjugate_gradient(
          -current.grad, hess, config.trust_region.subproblem_tol, radius);
        grasp_t const next_grasp = grasp_detail::apply_grasp_step(grasp, step);
        grasp_detail::grasp_evaluation_t const next_eval =
          grasp_detail::evaluate_grasp(problem, config, next_grasp);
        augmented_value_t const next =
          augmented_value(next_eval, dual_ineq, dual_eq, beta_ineq, beta_eq);
        Scalar const model = current.objective + current.grad.dot(step) +
          0.5 * step.dot(hess * step);
        Scalar const predicted = current.objective - model;
        Scalar rho = -std::numeric_limits<Scalar>::infinity();
        if (next_eval.finite() && std::isfinite(predicted) && predicted > 0.0) {
          rho = (current.objective - next.objective) / predicted;
        }

        if (rho > config.trust_region.improvement_tol) {
          grasp_detail::matrix7_t transport =
            grasp_detail::matrix7_t::Identity();
          Vector3 const half_rotation = -0.5 * step.segment<3>(3);
          Scalar const angle = half_rotation.norm();
          if (angle > 0.0) {
            transport.block<3, 3>(3, 3) = Quaternion {
              Eigen::AngleAxis<Scalar> {
                angle,
                half_rotation / angle}}.toRotationMatrix();
          }
          augmented_value_t const accepted =
            augmented_value(next_eval, dual_ineq, dual_eq, beta_ineq, beta_eq);
          grasp_detail::vector7_t const y =
            transport.transpose() * accepted.grad - current.grad;
          grasp_detail::vector7_t const y_transported = transport * y;
          grasp_detail::vector7_t const H_step = transport * hess * step;
          Scalar const y_step = y.dot(step);
          Scalar const step_H_step = step.dot(hess * step);
          if (std::abs(y_step) < 1e-8 || std::abs(step_H_step) < 1e-12) {
            hess.setIdentity();
          } else {
            hess = transport * hess * transport.transpose() +
              y_transported * y_transported.transpose() / y_step -
              H_step * H_step.transpose() / step_H_step;
          }
          grasp = next_grasp;
          eval = next_eval;
        }

        if (rho < config.trust_region.gain_ratio_lower) {
          radius *= config.trust_region.radius_reduction;
        } else if (rho > config.trust_region.gain_ratio_upper && boundary) {
          radius = std::min(
            config.trust_region.radius_expansion * radius,
            config.trust_region.radius_max);
        }
        if (step.norm() < config.trust_region.step_tol || !eval.finite()) {
          break;
        }
      }

      if (feasible(eval, config.alm)) {
        break;
      }
      dual_ineq = (dual_ineq + beta_ineq * eval.c_ineq).cwiseMax(0.0);
      dual_eq += beta_eq * eval.c_eq;
      Scalar const cur_ineq = max_inequality(eval.c_ineq);
      Scalar const cur_eq = max_equality(eval.c_eq);
      if (cur_ineq > prev_ineq / 2.0) {
        beta_ineq *= config.alm.inequality_penalty_increase;
      }
      if (cur_eq > prev_eq / 2.0) {
        beta_eq *= config.alm.equality_penalty_increase;
      }
      prev_ineq = cur_ineq;
      prev_eq = cur_eq;
    }

    bool const solved = feasible(eval, config.alm);
    planner_failure_t const failure =
      solved ? planner_failure_t {} : solve_failure(eval, config.alm);
    grasp_candidate_t candidate {
      .grasp = grasp,
      .score = eval.score,
      .contacts = std::move(eval.contacts),
      .solver =
        planner_solver_stats_t {
          .iters = total_iters,
          .converged = solved,
          .objective = eval.objective,
          .grad_norm = eval.grad.norm(),
        },
      .failure = failure,
    };
    return grasp_result_t {
      .status = solved ? solve_status_e::success : solve_status_e::infeasible,
      .candidates = {std::move(candidate)},
      .selected_index = solved ? std::optional<std::size_t> {0} : std::nullopt,
      .failure = failure,
    };
  }

}  // namespace stacking_core
