#pragma once

#include <stacking_core/posegen/types.hpp>

#include <Eigen/Core>

#include <vector>

namespace stacking_core::posegen_detail {

// The posegen force problem written directly over contact forces:
//
//   min  1/2 f' H f + q' f
//   s.t. sqrt(f_e0^2 + f_e1^2 + eps^2) <= friction_e * f_e2   for each contact
//
// One force per contact, expressed in that contact's frame with the normal
// last. The consensus split the force graph carries between the two
// node-local copies of a pair force is a decomposition device and has no
// place here, so this problem has half the unknowns of the graph form.
struct cone_qp_problem_t {
  Eigen::MatrixXd hessian;
  Eigen::VectorXd linear;
  std::vector<Scalar> friction;
  Scalar eps = 1e-2;
};

// Primal-dual interior point solve with Mehrotra predictor-corrector.
// `forces` is used as a starting point when it is the right size and strictly
// inside every cone, and receives the solution.
//
// Reported statistics follow the graph solver's conventions so the two are
// directly comparable: `iters` counts Newton steps, `primal_residual` is the
// complementarity gap, and `dual_residual` is the stationarity residual.
[[nodiscard]] posegen_force_solver_stats_t solve_cone_qp(
  cone_qp_problem_t const& problem,
  Eigen::VectorXd& forces,
  posegen_force_solver_config_t const& config);

}  // namespace stacking_core::posegen_detail
