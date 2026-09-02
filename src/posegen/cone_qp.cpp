#include "cone_qp.hpp"

#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

#include <algorithm>
#include <cmath>
#include <vector>

namespace stacking_core::posegen_detail {

namespace {

// Fraction of the distance to the cone boundary a step is allowed to take.
constexpr Scalar boundary_fraction = 0.995;

// Constraint value of one contact: negative strictly inside the cone.
Scalar constraint(
  Eigen::VectorXd const& f, std::size_t e, Scalar friction, Scalar eps) {
  auto const block = f.segment<3>(static_cast<Eigen::Index>(3 * e));
  return std::sqrt(
    block[0] * block[0] + block[1] * block[1] + eps * eps) -
    friction * block[2];
}

bool interior(cone_qp_problem_t const& problem, Eigen::VectorXd const& f) {
  for (std::size_t e = 0; e < problem.friction.size(); ++e) {
    if (!(constraint(f, e, problem.friction[e], problem.eps) < 0.0)) {
      return false;
    }
  }
  return true;
}

// Purely normal forces sit strictly inside every cone regardless of friction.
Eigen::VectorXd interior_start(cone_qp_problem_t const& problem) {
  Eigen::VectorXd f = Eigen::VectorXd::Zero(
    static_cast<Eigen::Index>(3 * problem.friction.size()));
  for (std::size_t e = 0; e < problem.friction.size(); ++e) {
    Scalar const friction = std::max(problem.friction[e], Scalar {1e-12});
    f[static_cast<Eigen::Index>(3 * e) + 2] =
      std::max(Scalar {1.0}, 4.0 * problem.eps / friction);
  }
  return f;
}

Vector3 constraint_grad(
  Eigen::VectorXd const& f, std::size_t e, Scalar friction, Scalar eps) {
  auto const block = f.segment<3>(static_cast<Eigen::Index>(3 * e));
  Scalar const r =
    std::sqrt(block[0] * block[0] + block[1] * block[1] + eps * eps);
  return Vector3 {block[0] / r, block[1] / r, -friction};
}

// Hessian of the constraint. Only the tangential block is non-zero, and it is
// positive semi-definite, so the condensed Newton matrix stays definite.
Matrix3 constraint_hess(
  Eigen::VectorXd const& f, std::size_t e, Scalar eps) {
  auto const block = f.segment<3>(static_cast<Eigen::Index>(3 * e));
  Scalar const r =
    std::sqrt(block[0] * block[0] + block[1] * block[1] + eps * eps);
  Matrix3 out = Matrix3::Zero();
  out(0, 0) = (1.0 - block[0] * block[0] / (r * r)) / r;
  out(1, 1) = (1.0 - block[1] * block[1] / (r * r)) / r;
  out(0, 1) = -block[0] * block[1] / (r * r * r);
  out(1, 0) = out(0, 1);
  return out;
}

}  // namespace

posegen_force_solver_stats_t solve_cone_qp(
  cone_qp_problem_t const& problem,
  Eigen::VectorXd& forces,
  posegen_force_solver_config_t const& config) {
  std::size_t const count = problem.friction.size();
  Eigen::Index const dim = static_cast<Eigen::Index>(3 * count);
  posegen_force_solver_stats_t stats;
  if (count == 0) {
    forces.resize(0);
    stats.converged = true;
    return stats;
  }
  if (forces.size() != dim || !interior(problem, forces)) {
    forces = interior_start(problem);
  }

  Eigen::VectorXd multiplier = Eigen::VectorXd::Ones(
    static_cast<Eigen::Index>(count));
  Eigen::VectorXd slack(static_cast<Eigen::Index>(count));
  // Each constraint touches only its own contact, so the constraint Jacobian
  // is stored as one 3-vector per contact rather than a mostly empty matrix.
  // Every product it appears in is then linear in the number of contacts.
  std::vector<Vector3> gradient(count, Vector3::Zero());
  Eigen::VectorXd residual(dim);

  // The Newton matrix couples two contacts only when they share a body, so it
  // inherits the Hessian's sparsity. The pattern never changes across Newton
  // steps, so it is analyzed once and only the values are refilled.
  std::vector<Eigen::Triplet<Scalar>> base_triplets;
  for (Eigen::Index col = 0; col < dim; ++col) {
    for (Eigen::Index row = 0; row < dim; ++row) {
      if (problem.hessian(row, col) != 0.0) {
        base_triplets.emplace_back(row, col, problem.hessian(row, col));
      }
    }
  }
  // Only the per-contact diagonal blocks change between Newton steps, so the
  // matrix is built once and those entries are then written in place.
  for (std::size_t e = 0; e < count; ++e) {
    Eigen::Index const base_index = static_cast<Eigen::Index>(3 * e);
    for (Eigen::Index j = 0; j < 3; ++j) {
      for (Eigen::Index i = 0; i < 3; ++i) {
        base_triplets.emplace_back(base_index + i, base_index + j, 0.0);
      }
    }
  }
  Eigen::SparseMatrix<Scalar> newton(dim, dim);
  newton.setFromTriplets(base_triplets.begin(), base_triplets.end());
  newton.makeCompressed();

  std::vector<Matrix3> hessian_diagonal(count);
  std::vector<Eigen::Index> diagonal_index(9 * count, -1);
  for (std::size_t e = 0; e < count; ++e) {
    Eigen::Index const base_index = static_cast<Eigen::Index>(3 * e);
    hessian_diagonal[e] = problem.hessian.block<3, 3>(base_index, base_index);
    for (Eigen::Index j = 0; j < 3; ++j) {
      for (Eigen::Index i = 0; i < 3; ++i) {
        for (Eigen::Index k = newton.outerIndexPtr()[base_index + j];
             k < newton.outerIndexPtr()[base_index + j + 1]; ++k) {
          if (newton.innerIndexPtr()[k] == base_index + i) {
            diagonal_index[9 * e + 3 * j + i] = k;
            break;
          }
        }
      }
    }
  }

  Eigen::SimplicialLLT<Eigen::SparseMatrix<Scalar>> llt;
  llt.analyzePattern(newton);

  auto refresh = [&]() {
    residual = problem.hessian * forces + problem.linear;
    for (std::size_t e = 0; e < count; ++e) {
      Eigen::Index const row = static_cast<Eigen::Index>(e);
      slack[row] = -constraint(forces, e, problem.friction[e], problem.eps);
      gradient[e] =
        constraint_grad(forces, e, problem.friction[e], problem.eps);
      residual.segment<3>(static_cast<Eigen::Index>(3 * e)) +=
        multiplier[row] * gradient[e];
    }
  };

  // Largest step that keeps every cone and every multiplier strictly positive.
  auto max_step = [&](Eigen::VectorXd const& d_force,
                      Eigen::VectorXd const& d_multiplier) {
    Scalar alpha = 1.0;
    for (Eigen::Index i = 0; i < multiplier.size(); ++i) {
      if (d_multiplier[i] < 0.0) {
        alpha = std::min(
          alpha, -boundary_fraction * multiplier[i] / d_multiplier[i]);
      }
    }
    for (int trial = 0; trial < 64; ++trial) {
      Eigen::VectorXd const candidate = forces + alpha * d_force;
      bool ok = true;
      for (std::size_t e = 0; e < count && ok; ++e) {
        Scalar const value = -constraint(
          candidate, e, problem.friction[e], problem.eps);
        ok = value > (1.0 - boundary_fraction) *
          slack[static_cast<Eigen::Index>(e)];
      }
      if (ok) {
        return alpha;
      }
      alpha *= 0.5;
    }
    return Scalar {0.0};
  };

  refresh();
  for (int iter = 0; iter < config.max_iters; ++iter) {
    stats.iters = iter + 1;
    Scalar const gap = multiplier.dot(slack) / static_cast<Scalar>(count);
    Scalar const dual_norm = residual.cwiseAbs().maxCoeff();
    Scalar const scale = std::max(
      Scalar {1.0}, (problem.hessian * forces).cwiseAbs().maxCoeff());
    stats.primal_residual = gap;
    stats.dual_residual = dual_norm;
    if (gap < config.tol_abs &&
        dual_norm < config.tol_abs + config.tol_rel * scale) {
      stats.converged = true;
      break;
    }

    // Condensed Newton matrix, shared by the predictor and the corrector.
    for (std::size_t e = 0; e < count; ++e) {
      Eigen::Index const row = static_cast<Eigen::Index>(e);
      Matrix3 const block = hessian_diagonal[e] +
        multiplier[row] * constraint_hess(forces, e, problem.eps) +
        (multiplier[row] / slack[row]) * gradient[e] *
          gradient[e].transpose();
      for (Eigen::Index j = 0; j < 3; ++j) {
        for (Eigen::Index i = 0; i < 3; ++i) {
          newton.valuePtr()[diagonal_index[9 * e + 3 * j + i]] =
            block(i, j);
        }
      }
    }

    llt.factorize(newton);
    if (llt.info() != Eigen::Success) {
      return stats;
    }
    Eigen::VectorXd const base =
      -(problem.hessian * forces + problem.linear);

    auto direction = [&](Scalar barrier) {
      Eigen::VectorXd rhs = base;
      if (barrier > 0.0) {
        for (std::size_t e = 0; e < count; ++e) {
          rhs.segment<3>(static_cast<Eigen::Index>(3 * e)) -=
            (barrier / slack[static_cast<Eigen::Index>(e)]) * gradient[e];
        }
      }
      Eigen::VectorXd const d_force = llt.solve(rhs);
      Eigen::VectorXd d_slack(static_cast<Eigen::Index>(count));
      for (std::size_t e = 0; e < count; ++e) {
        d_slack[static_cast<Eigen::Index>(e)] = -gradient[e].dot(
          d_force.segment<3>(static_cast<Eigen::Index>(3 * e)));
      }
      Eigen::VectorXd const d_multiplier =
        ((barrier - (multiplier.array() * slack.array()) -
          multiplier.array() * d_slack.array()) /
         slack.array())
          .matrix();
      return std::pair {d_force, d_multiplier};
    };

    // Predictor: how much progress a pure Newton step would make.
    auto const [affine_force, affine_multiplier] = direction(0.0);
    Scalar const affine_alpha = max_step(affine_force, affine_multiplier);
    Eigen::VectorXd affine_slack(static_cast<Eigen::Index>(count));
    for (std::size_t e = 0; e < count; ++e) {
      affine_slack[static_cast<Eigen::Index>(e)] = -constraint(
        (forces + affine_alpha * affine_force).eval(), e,
        problem.friction[e], problem.eps);
    }
    Scalar const affine_gap =
      (multiplier + affine_alpha * affine_multiplier).dot(affine_slack) /
      static_cast<Scalar>(count);

    // Mehrotra's centering rule: take a short step when the predictor was
    // unproductive, and a nearly pure Newton step when it was good.
    Scalar const sigma = gap > 0.0
      ? std::clamp(
          std::pow(affine_gap / gap, 3.0), Scalar {1e-8}, Scalar {1.0})
      : Scalar {1e-8};
    auto const [d_force, d_multiplier] = direction(sigma * gap);

    Scalar const alpha = max_step(d_force, d_multiplier);
    if (alpha <= 0.0) {
      return stats;
    }
    forces += alpha * d_force;
    multiplier += alpha * d_multiplier;
    multiplier = multiplier.cwiseMax(Scalar {1e-14});
    refresh();
  }
  return stats;
}

}  // namespace stacking_core::posegen_detail
