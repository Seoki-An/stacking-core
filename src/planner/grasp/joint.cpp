#include "evaluation.hpp"

#include <stacking_core/optimization/truncated_conjugate_gradient.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace stacking_core {
  namespace {

    struct augmented_joint_t {
      Scalar objective = 0.0;
      Eigen::VectorXd grad;
      Eigen::VectorXd active_ineq;
    };

    struct joint_limit_constraint_t {
      Eigen::Index index = 0;
      Scalar coefficient = 1.0;
      Scalar offset = 0.0;
      Scalar lower = 0.0;
      Scalar upper = 0.0;
    };

    Eigen::Index phase_offset(std::size_t phase, Eigen::Index dof) {
      return phase == 0 ? 0
                        : dof + 1 + static_cast<Eigen::Index>(phase - 1) * dof;
    }

    std::vector<joint_limit_constraint_t> joint_limits(
      KinematicModel const& model) {
      std::vector<joint_limit_constraint_t> out;
      Eigen::VectorXd const zero = Eigen::VectorXd::Zero(
        static_cast<Eigen::Index>(model.degreeOfFreedomCount()));
      for (std::size_t i = 0; i < model.jointCount(); ++i) {
        kinematic_joint_t const& joint = model.joint(i);
        if (!joint.limit.has_value()) {
          continue;
        }
        std::optional<std::size_t> const dof_index =
          model.degreeOfFreedomIndex(joint.id);
        if (!dof_index.has_value()) {
          continue;
        }
        Eigen::VectorXd unit = zero;
        unit(static_cast<Eigen::Index>(*dof_index)) = 1.0;
        Scalar const offset = model.jointPosition(joint.id, zero);
        Scalar const coefficient = model.jointPosition(joint.id, unit) - offset;
        out.push_back(joint_limit_constraint_t {
          .index = static_cast<Eigen::Index>(*dof_index),
          .coefficient = coefficient,
          .offset = offset,
          .lower = joint.limit->lower,
          .upper = joint.limit->upper,
        });
      }
      return out;
    }

    Vector3 vex(Matrix3 const& matrix) {
      return Vector3 {-matrix(1, 2), matrix(0, 2), -matrix(0, 1)};
    }

    grasp_detail::joint_evaluation_t evaluate_joint_impl(
      joint_grasp_problem_t const& problem,
      grasp_generation_config_t const& config,
      Eigen::VectorXd const& variables) {
      Eigen::Index const dof = static_cast<Eigen::Index>(
        problem.initial_states.front().model().degreeOfFreedomCount());
      Eigen::Index const opening_index = dof;
      Eigen::Index const dim =
        static_cast<Eigen::Index>(problem.initial_states.size()) * dof + 1;
      std::vector<KinematicSnapshot> snapshots;
      std::vector<pose_t> grasp_poses;
      std::vector<Matrix6X> grasp_jacs;
      snapshots.reserve(problem.initial_states.size());
      grasp_poses.reserve(problem.initial_states.size());
      grasp_jacs.reserve(problem.initial_states.size());
      for (std::size_t phase = 0; phase < problem.initial_states.size();
           ++phase) {
        KinematicState state = problem.initial_states[phase];
        state.setPositions(variables.segment(phase_offset(phase, dof), dof));
        snapshots.push_back(forward_kinematics(state));
        grasp_poses.push_back(compose(
          snapshots.back().frameFromLink(problem.grasp_link),
          problem.link_from_grasp));
        grasp_jacs.push_back(grasp_detail::body_pose_jacobian(
          snapshots.back(), problem.grasp_link, problem.link_from_grasp));
      }

      grasp_t const grasp {
        .frame_from_grasp = grasp_poses.front(),
        .opening = variables(opening_index),
      };
      grasp_detail::grasp_evaluation_t base =
        grasp_detail::evaluate_grasp(problem.grasp, config, grasp);
      Eigen::MatrixXd grasp_map = Eigen::MatrixXd::Zero(7, dim);
      grasp_map.topRows(6).leftCols(dof) = grasp_jacs.front();
      grasp_map(6, opening_index) = 1.0;

      grasp_detail::joint_evaluation_t out;
      out.objective = base.objective;
      out.score = base.score;
      out.grad = grasp_map.transpose() * base.grad;
      out.grasp = grasp;
      out.contacts = std::move(base.contacts);

      std::vector<joint_limit_constraint_t> const limits =
        joint_limits(problem.initial_states.front().model());
      Eigen::Index const n_joint_ineq = 2 *
        static_cast<Eigen::Index>(limits.size()) *
        static_cast<Eigen::Index>(problem.initial_states.size());
      out.c_ineq = Eigen::VectorXd::Zero(base.c_ineq.size() + n_joint_ineq);
      out.jac_ineq = Eigen::MatrixXd::Zero(out.c_ineq.size(), dim);
      out.c_ineq.head(base.c_ineq.size()) = base.c_ineq;
      out.jac_ineq.topRows(base.c_ineq.size()) = base.jac_ineq * grasp_map;
      Eigen::Index row = base.c_ineq.size();
      for (std::size_t phase = 0; phase < problem.initial_states.size();
           ++phase) {
        Eigen::Index const offset = phase_offset(phase, dof);
        for (joint_limit_constraint_t const& limit : limits) {
          out.c_ineq(row) =
            limit.coefficient * variables(offset + limit.index) + limit.offset -
            limit.upper;
          out.jac_ineq(row, offset + limit.index) = limit.coefficient;
          ++row;
        }
        for (joint_limit_constraint_t const& limit : limits) {
          out.c_ineq(row) =
            -limit.coefficient * variables(offset + limit.index) -
            limit.offset + limit.lower;
          out.jac_ineq(row, offset + limit.index) = -limit.coefficient;
          ++row;
        }
      }

      Eigen::Index const extra_count =
        static_cast<Eigen::Index>(problem.initial_states.size() - 1);
      out.c_eq = Eigen::VectorXd::Zero(base.c_eq.size() + 2 * extra_count);
      out.jac_eq = Eigen::MatrixXd::Zero(out.c_eq.size(), dim);
      out.c_eq.head(base.c_eq.size()) = base.c_eq;
      out.jac_eq.topRows(base.c_eq.size()) = base.jac_eq * grasp_map;

      phase_scene_t const& ref_phase = problem.grasp.phases.front();
      pose_t const& frame_from_ref_target =
        ref_phase.scene.body(ref_phase.target).frameFromBody();
      for (Eigen::Index extra = 0; extra < extra_count; ++extra) {
        std::size_t const phase = static_cast<std::size_t>(extra + 1);
        phase_scene_t const& phase_scene = problem.grasp.phases[phase];
        pose_t const ref_from_phase = compose(
          frame_from_ref_target,
          inverse(phase_scene.scene.body(phase_scene.target).frameFromBody()));
        pose_t const ref_from_extra =
          compose(ref_from_phase, grasp_poses[phase]);
        Matrix6X J_extra = grasp_jacs[phase];
        J_extra.topRows<3>() =
          ref_from_phase.orientation.toRotationMatrix() * J_extra.topRows<3>();

        Vector3 const p_err =
          ref_from_extra.position - grasp_poses.front().position;
        Matrix3 const R_err = (ref_from_extra.orientation.conjugate() *
                               grasp_poses.front().orientation)
                                .toRotationMatrix();
        Vector3 const R_err_vec = 0.5 * vex(R_err - R_err.transpose());
        Eigen::Index const eq_row = base.c_eq.size() + 2 * extra;
        out.c_eq(eq_row) = 0.5 * p_err.squaredNorm();
        out.c_eq(eq_row + 1) = 0.5 * (Matrix3::Identity() - R_err).trace();

        out.jac_eq.block(eq_row, 0, 1, dof) =
          -p_err.transpose() * grasp_jacs.front().topRows<3>();
        Eigen::Index const offset = phase_offset(phase, dof);
        out.jac_eq.block(eq_row, offset, 1, dof) =
          p_err.transpose() * J_extra.topRows<3>();
        out.jac_eq.block(eq_row + 1, 0, 1, dof) =
          R_err_vec.transpose() * grasp_jacs.front().bottomRows<3>();
        out.jac_eq.block(eq_row + 1, offset, 1, dof) =
          -R_err_vec.transpose() * J_extra.bottomRows<3>();
      }
      return out;
    }

    augmented_joint_t augmented_value(
      grasp_detail::joint_evaluation_t const& eval,
      Eigen::VectorXd const& dual_ineq, Eigen::VectorXd const& dual_eq,
      Scalar beta_ineq, Scalar beta_eq) {
      Eigen::VectorXd const active =
        (dual_ineq + beta_ineq * eval.c_ineq).cwiseMax(0.0);
      return augmented_joint_t {
        .objective = eval.objective +
          (active.squaredNorm() - dual_ineq.squaredNorm()) / (2.0 * beta_ineq) +
          dual_eq.dot(eval.c_eq) + 0.5 * beta_eq * eval.c_eq.squaredNorm(),
        .grad = eval.grad + eval.jac_ineq.transpose() * active +
          eval.jac_eq.transpose() * dual_eq +
          beta_eq * eval.jac_eq.transpose() * eval.c_eq,
        .active_ineq = active,
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
      grasp_detail::joint_evaluation_t const& eval,
      grasp_alm_config_t const& config) {
      return eval.finite() &&
        max_inequality(eval.c_ineq) <= config.inequality_tol &&
        max_equality(eval.c_eq) <= config.equality_tol;
    }

    std::optional<planner_failure_t> validate_joint_problem(
      joint_grasp_problem_t const& problem,
      grasp_generation_config_t const& config) {
      if (
        auto const failure =
          grasp_detail::validate_grasp_problem(problem.grasp, config)) {
        return failure;
      }
      if (problem.initial_states.size() != problem.grasp.phases.size()) {
        return planner_failure_t {
          .code = "joint_phase_count_mismatch",
          .message = "joint grasp requires one manipulator state per phase",
          .retryable = false,
        };
      }
      if (
        problem.initial_states.empty() || !is_valid(problem.link_from_grasp)) {
        return planner_failure_t {
          .code = "invalid_joint_grasp_model",
          .message = "joint grasp link transform must be valid",
          .retryable = false,
        };
      }
      auto const model = problem.initial_states.front().modelPtr();
      if (model->findLink(problem.grasp_link) == nullptr) {
        return planner_failure_t {
          .code = "invalid_grasp_link",
          .message = "joint grasp link does not belong to the manipulator",
          .retryable = false,
        };
      }
      for (KinematicState const& state : problem.initial_states) {
        if (state.modelPtr() != model || !state.positions().allFinite()) {
          return planner_failure_t {
            .code = "incompatible_joint_states",
            .message =
              "joint grasp states must use one model and finite positions",
            .retryable = false,
          };
        }
      }
      return std::nullopt;
    }

  }  // namespace

  bool grasp_detail::joint_evaluation_t::finite() const noexcept {
    return std::isfinite(objective) && std::isfinite(score) &&
      grad.allFinite() && c_ineq.allFinite() && c_eq.allFinite() &&
      jac_ineq.allFinite() && jac_eq.allFinite();
  }

  grasp_detail::joint_evaluation_t grasp_detail::evaluate_joint(
    joint_grasp_problem_t const& problem,
    grasp_generation_config_t const& config, Eigen::VectorXd const& variables) {
    return evaluate_joint_impl(problem, config, variables);
  }

  joint_grasp_result_t solve_joint_grasp(
    joint_grasp_problem_t const& problem,
    grasp_generation_config_t const& config) {
    if (auto const failure = validate_joint_problem(problem, config)) {
      return joint_grasp_result_t {
        .status = solve_status_e::invalid_problem,
        .candidates = {},
        .selected_index = std::nullopt,
        .failure = *failure,
      };
    }

    Eigen::Index const dof = static_cast<Eigen::Index>(
      problem.initial_states.front().model().degreeOfFreedomCount());
    Eigen::Index const dim =
      static_cast<Eigen::Index>(problem.initial_states.size()) * dof + 1;
    Eigen::VectorXd variables = Eigen::VectorXd::Zero(dim);
    variables.head(dof) = problem.initial_states.front().positions();
    variables(dof) = problem.grasp.seed.opening;
    for (std::size_t phase = 1; phase < problem.initial_states.size();
         ++phase) {
      variables.segment(phase_offset(phase, dof), dof) =
        problem.initial_states[phase].positions();
    }

    grasp_detail::joint_evaluation_t eval =
      grasp_detail::evaluate_joint(problem, config, variables);
    Eigen::VectorXd dual_ineq = Eigen::VectorXd::Zero(eval.c_ineq.size());
    Eigen::VectorXd dual_eq = Eigen::VectorXd::Zero(eval.c_eq.size());
    Scalar beta_ineq = config.alm.inequality_penalty_init;
    Scalar beta_eq = config.alm.equality_penalty_init;
    Scalar prev_ineq = std::numeric_limits<Scalar>::infinity();
    Scalar prev_eq = std::numeric_limits<Scalar>::infinity();
    int total_iters = 0;

    for (int alm_iter = 0; alm_iter < config.alm.max_iters; ++alm_iter) {
      Scalar radius = config.trust_region.radius_init;
      for (int iter = 0; iter < config.trust_region.max_iters; ++iter) {
        ++total_iters;
        augmented_joint_t const current =
          augmented_value(eval, dual_ineq, dual_eq, beta_ineq, beta_eq);
        Eigen::MatrixXd hess = 1e-6 * Eigen::MatrixXd::Identity(dim, dim);
        hess.noalias() += beta_eq * eval.jac_eq.transpose() * eval.jac_eq;
        Eigen::MatrixXd active_jac = eval.jac_ineq;
        for (Eigen::Index row = 0; row < active_jac.rows(); ++row) {
          if (current.active_ineq(row) <= 0.0) {
            active_jac.row(row).setZero();
          }
        }
        hess.noalias() += beta_ineq * active_jac.transpose() * active_jac;
        auto const [step, boundary] = truncated_conjugate_gradient(
          -current.grad, hess, config.trust_region.subproblem_tol, radius);
        Eigen::VectorXd const next_variables = variables + step;
        grasp_detail::joint_evaluation_t const next_eval =
          grasp_detail::evaluate_joint(problem, config, next_variables);
        augmented_joint_t const next =
          augmented_value(next_eval, dual_ineq, dual_eq, beta_ineq, beta_eq);
        Scalar const model = current.objective + current.grad.dot(step) +
          0.5 * step.dot(hess * step);
        Scalar const predicted = current.objective - model;
        Scalar rho = -std::numeric_limits<Scalar>::infinity();
        if (next_eval.finite() && std::isfinite(predicted) && predicted > 0.0) {
          rho = (current.objective - next.objective) / predicted;
        }
        if (rho > config.trust_region.improvement_tol) {
          variables = next_variables;
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
    planner_failure_t const failure = solved
      ? planner_failure_t {}
      : planner_failure_t {
          .code = eval.finite() ? "joint_grasp_constraints_infeasible"
                                : "nonfinite_joint_grasp_evaluation",
          .message = "joint-space grasp did not satisfy its constraints",
          .retryable = true,
        };
    std::vector<Eigen::VectorXd> positions;
    positions.reserve(problem.initial_states.size());
    for (std::size_t phase = 0; phase < problem.initial_states.size();
         ++phase) {
      positions.push_back(variables.segment(phase_offset(phase, dof), dof));
    }
    joint_grasp_candidate_t candidate {
      .grasp =
        grasp_candidate_t {
          .grasp = eval.grasp,
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
        },
      .positions = std::move(positions),
    };
    return joint_grasp_result_t {
      .status = solved ? solve_status_e::success : solve_status_e::infeasible,
      .candidates = {std::move(candidate)},
      .selected_index = solved ? std::optional<std::size_t> {0} : std::nullopt,
      .failure = failure,
    };
  }

}  // namespace stacking_core
