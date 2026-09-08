#pragma once

#include <stacking_core/simulation/types.hpp>

#include <Eigen/Core>

#include <compare>
#include <functional>
#include <map>
#include <vector>

namespace stacking_core::simulation_detail {

using vector_x_t = Eigen::VectorXd;
using matrix_x6_t = Eigen::Matrix<Scalar, Eigen::Dynamic, 6>;

enum class factor_kind_e {
  contact_3d,
  contact_4d,
};

struct factor_id_t {
  factor_kind_e kind = factor_kind_e::contact_3d;
  EntityId first;
  EntityId second;
  GeometryId first_geometry;
  GeometryId second_geometry;

  auto operator<=>(factor_id_t const&) const = default;
};

struct constraint_factor_t {
  factor_id_t id;
  bool hard_inequality = false;
  std::vector<EntityId> entities;
  std::vector<matrix_x6_t> jacobians;
  vector_x_t error;
  std::function<vector_x_t(vector_x_t const&)> projector;
};

// Per-body dynamics. The equations of motion are the objective of this solve,
// not one of its constraints, so they enter the velocity update directly
// rather than as a factor: the update solves
//   (mass + beta * sum C' C) v = momentum + sum C' (...)
// whose fixed point is mass * v = momentum + sum C' impulse.
struct constraint_node_t {
  // Effective dynamics matrix M + dt D (physical mass when D is zero).
  Matrix6 mass = Matrix6::Identity();
  // Physical M * v0 + force * dt, before damping and contact impulses.
  Vector6 momentum = Vector6::Zero();
  // Warm start on entry, solution on exit.
  Vector6 velocity = Vector6::Zero();
};

class ConstraintSolver {
public:
  void clear_factors();
  void clear_warm_start();
  void add_factor(constraint_factor_t factor);

  [[nodiscard]] simulation_solver_stats_t solve(
    std::map<EntityId, constraint_node_t>& nodes,
    simulation_solver_config_t const& config,
    Scalar dynamics_scale);

private:
  struct warm_factor_t {
    vector_x_t impulse;
    std::vector<vector_x_t> auxiliary;
  };

  std::vector<constraint_factor_t> factors_;
  std::map<factor_id_t, warm_factor_t> warm_start_;
  Scalar warm_beta_ = -1.0;
};

}  // namespace stacking_core::simulation_detail
