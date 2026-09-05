#include "constraint_solver.hpp"

#include <stacking_core/contact/friction.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

using namespace stacking_core;
using namespace stacking_core::simulation_detail;

void require(
  bool value, std::string const& message,
  std::source_location loc = std::source_location::current()) {
  if (!value) {
    throw std::runtime_error(
      "constraint solver line " + std::to_string(loc.line()) + ": " + message);
  }
}

constraint_node_t scalar_node(Scalar mass, Scalar free_vel) {
  constraint_node_t node;
  node.mass(0, 0) = mass;
  node.momentum[0] = mass * free_vel;
  node.velocity[0] = free_vel;
  return node;
}

constraint_factor_t scalar_contact(
  std::vector<EntityId> ids, std::vector<Scalar> signs, Scalar& impulse) {
  constraint_factor_t factor;
  factor.id.first = ids.front();
  if (ids.size() > 1) {
    factor.id.second = ids.back();
  }
  factor.hard_inequality = true;
  factor.entities = std::move(ids);
  factor.error = vector_x_t::Zero(1);
  for (Scalar sign : signs) {
    matrix_x6_t J = matrix_x6_t::Zero(1, 6);
    J(0, 0) = sign;
    factor.jacobians.push_back(std::move(J));
  }
  factor.projector = [&impulse](vector_x_t const& arg) -> vector_x_t {
    impulse = std::max(Scalar {0.0}, arg[0]);
    return vector_x_t::Constant(1, impulse);
  };
  return factor;
}

simulation_solver_config_t fixed_config(Scalar beta) {
  simulation_solver_config_t config;
  config.damping = 1.0;
  config.beta_init = beta;
  config.beta_min = beta;
  config.beta_max = beta;
  config.tol_abs = 1e-10;
  config.tol_rel = 0.0;
  config.stagnation_window = 0;
  return config;
}

void separating_contact() {
  // min 0.5 * (v - 1)^2, v >= 0: v*=1, lambda*=0. A contact with
  // nonzero feasible velocity must have zero momentum residual at its solution.
  ConstraintSolver solver;
  Scalar impulse = 0.0;
  solver.add_factor(scalar_contact({EntityId {1}}, {1.0}, impulse));
  std::map<EntityId, constraint_node_t> nodes {
    {EntityId {1}, scalar_node(1.0, 1.0)}};
  auto const stats = solver.solve(nodes, fixed_config(1.0), 1.0);
  require(stats.converged, "separating contact must converge");
  require(stats.iters < 100, "separating contact must not burn the budget");
  require(std::abs(nodes.begin()->second.velocity[0] - 1.0) < 1e-9,
    "separating motion must be preserved");
  require(impulse == 0.0, "separating contact must not apply impulse");
  require(stats.dual_residual < 1e-10, "momentum balance at separation");
}

void budget_residual() {
  ConstraintSolver solver;
  Scalar impulse = 0.0;
  solver.add_factor(scalar_contact({EntityId {1}}, {1.0}, impulse));
  std::map<EntityId, constraint_node_t> nodes {
    {EntityId {1}, scalar_node(1.0, 1.0)}};
  auto config = fixed_config(1.0);
  config.max_iters = 1;
  auto const stats = solver.solve(nodes, config, 1.0);
  auto const& node = nodes.begin()->second;
  Scalar const expected = std::abs(node.velocity[0] - 1.0 - impulse);
  require(!stats.converged && expected > 0.1, "one sweep is incomplete");
  require(std::abs(stats.dual_residual - expected) < 1e-14,
    "reported residual must describe the returned velocity, even at the cap");
}

void sliding_contact() {
  ConstraintSolver solver;
  Vector3 impulse = Vector3::Zero();
  constraint_factor_t factor;
  factor.id.first = EntityId {1};
  factor.entities = {EntityId {1}};
  factor.hard_inequality = true;
  factor.error = Vector3::Zero();
  matrix_x6_t J = matrix_x6_t::Zero(3, 6);
  J.leftCols<3>().setIdentity();
  factor.jacobians.push_back(J);
  factor.projector = [&impulse](vector_x_t const& arg) -> vector_x_t {
    impulse = project_coulomb_impulse(arg.head<3>(), 0.6);
    return impulse;
  };
  solver.add_factor(std::move(factor));
  constraint_node_t node;
  node.velocity.head<3>() = Vector3 {0.1, 0.0, -0.04905};
  node.momentum = node.velocity;
  std::map<EntityId, constraint_node_t> nodes {{EntityId {1}, node}};
  auto const stats = solver.solve(nodes, fixed_config(1.0), 1.0);
  // Radial Coulomb law: cancel the incoming normal speed and subtract mu*n
  // from the sliding speed. This is not an associated SOCP projection.
  Vector3 const expected {0.1 - 0.6 * 0.04905, 0.0, 0.0};
  Vector3 const vel = nodes.begin()->second.velocity.head<3>();
  require(stats.converged, "sliding contact must converge");
  require((vel - expected).norm() < 1e-9, "analytical Coulomb sliding velocity");
  require((vel - node.momentum.head<3>() - impulse).norm() < 1e-9,
    "sliding momentum balance");
}

void cold_start_guard() {
  ConstraintSolver solver;
  Scalar impulse = 0.0;
  auto factor = scalar_contact({EntityId {1}}, {1.0}, impulse);
  factor.jacobians.front() *= 0.005;
  solver.add_factor(std::move(factor));
  std::map<EntityId, constraint_node_t> nodes {
    {EntityId {1}, scalar_node(1.0, -0.04905)}};
  simulation_solver_config_t config;
  config.max_iters = 1;
  auto const stats = solver.solve(nodes, config, 1.0);
  require(stats.primal_residual == 0.0 && stats.dual_residual < config.tol_abs,
    "first sweep can pass both residual thresholds with cold auxiliaries");
  require(!stats.converged, "first sweep has not resolved the incoming contact");
}

void moving_pair() {
  // Two masses approaching along one axis: exact inelastic common velocity
  // is (1*1 + 3*0)/(1+3)=0.25, with equal/opposite impulse 0.75.
  ConstraintSolver solver;
  Scalar impulse = 0.0;
  solver.add_factor(scalar_contact(
    {EntityId {1}, EntityId {2}}, {-1.0, 1.0}, impulse));
  std::map<EntityId, constraint_node_t> nodes {
    {EntityId {1}, scalar_node(1.0, 1.0)},
    {EntityId {2}, scalar_node(3.0, 0.0)}};
  auto config = fixed_config(1.0);
  config.damping = 2.0;  // Unit contact scale for the two unit Jacobians.
  auto const stats = solver.solve(nodes, config, 1.0);
  require(stats.converged, "moving pair must converge");
  require(std::abs(impulse - 0.75) < 1e-8, "pair impulse analytical solution");
  for (auto const& [id, node] : nodes) {
    (void)id;
    require(std::abs(node.velocity[0] - 0.25) < 1e-8,
      "pair velocity analytical solution");
  }

  // A penalty change must preserve this nonzero-velocity/contact-impulse
  // fixed point. Auxiliary values need conversion to the new beta.
  config.beta_min = config.beta_max = 10.0;
  config.max_iters = 1;
  auto const warm_stats = solver.solve(nodes, config, 1.0);
  require(warm_stats.dual_residual < 1e-8,
    "changing cached beta must preserve momentum balance");
  require(std::abs(impulse - 0.75) < 1e-8,
    "changing beta must not rescale the contact impulse");
  for (auto const& [id, node] : nodes) {
    (void)id;
    require(std::abs(node.velocity[0] - 0.25) < 1e-8,
      "changing beta must preserve the moving fixed point");
  }
}

void adaptive_transition() {
  ConstraintSolver solver;
  Scalar impulse = 0.0;
  solver.add_factor(scalar_contact({EntityId {1}}, {1.0}, impulse));
  std::map<EntityId, constraint_node_t> nodes {
    {EntityId {1}, scalar_node(1.0, -1.0)}};
  auto config = fixed_config(0.01);
  config.beta_min = 1e-4;
  config.beta_max = 1e4;
  config.beta_update_interval = 1;
  config.max_iters = 3;
  config.tol_abs = config.tol_rel = 0.0;

  // Hand-evaluated first two sweeps, followed by one penalty change. With
  // scalar unit mass/J, these require no graph assembly or factorization.
  Scalar const beta = 0.01;
  Scalar const v_1 = -1.0 / (1.0 + beta);
  Scalar const lam_2 = -beta * v_1;
  Scalar const v_2 = (-1.0 + beta * v_1 + 2.0 * lam_2) / (1.0 + beta);
  Scalar const r_2 = lam_2 / beta;
  Scalar const s_2 = std::abs(v_2 + 1.0 - lam_2);
  Scalar const beta_3 = beta * std::sqrt(r_2 / std::abs(v_2) / s_2);
  Scalar const lam_3 = lam_2 - beta_3 * v_2;
  Scalar const v_3 =
    (-1.0 + beta_3 * v_2 + 2.0 * lam_3 - lam_2) / (1.0 + beta_3);
  auto const stats = solver.solve(nodes, config, 1.0);
  require(std::abs(impulse - lam_3) < 1e-13,
    "projection must use the converted auxiliary and new beta");
  require(std::abs(nodes.begin()->second.velocity[0] - v_3) < 1e-13,
    "RHS and factorization must use the same beta");
  require(std::abs(stats.dual_residual - std::abs(v_3 + 1.0 - lam_3)) < 1e-13,
    "adaptive residual must match the returned state");
}

}  // namespace

int main() {
  int failures = 0;
  auto run = [&](char const* name, auto const& test) {
    try {
      test();
    } catch (std::exception const& error) {
      std::cerr << name << ": " << error.what() << '\n';
      ++failures;
    }
  };
  run("separating_contact", separating_contact);
  run("budget_residual", budget_residual);
  run("sliding_contact", sliding_contact);
  run("cold_start_guard", cold_start_guard);
  run("moving_pair", moving_pair);
  run("adaptive_transition", adaptive_transition);
  return failures == 0 ? 0 : 1;
}
