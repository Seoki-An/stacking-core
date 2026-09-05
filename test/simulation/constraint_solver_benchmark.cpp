#include "constraint_solver.hpp"

#include <stacking_core/contact/friction.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

namespace {

using namespace stacking_core;
using namespace stacking_core::simulation_detail;

struct problem_t {
  std::map<EntityId, constraint_node_t> nodes;
  std::map<EntityId, Vector6> expected;
  std::vector<constraint_factor_t> factors;
  std::vector<std::shared_ptr<Vector3>> impulses;
  Scalar friction = 0.0;
};

void add_contact(problem_t& problem, std::vector<EntityId> ids) {
  auto impulse = std::make_shared<Vector3>(Vector3::Zero());
  constraint_factor_t factor;
  factor.id.first = ids.front();
  if (ids.size() == 2) {
    factor.id.second = ids.back();
  }
  factor.hard_inequality = true;
  factor.entities = std::move(ids);
  factor.error = Vector3::Zero();
  for (std::size_t i = 0; i < factor.entities.size(); ++i) {
    matrix_x6_t J = matrix_x6_t::Zero(3, 6);
    // Same dt-scaled contact coordinates as Simulator, with zero lever arms.
    Scalar const sign = factor.entities.size() == 2 && i == 0 ? -1.0 : 1.0;
    J.leftCols<3>() = sign * 0.005 * Matrix3::Identity();
    factor.jacobians.push_back(std::move(J));
  }
  factor.projector = [impulse, mu = problem.friction](vector_x_t const& arg) {
    *impulse = project_coulomb_impulse(arg.head<3>(), mu);
    return vector_x_t {*impulse};
  };
  problem.factors.push_back(std::move(factor));
  problem.impulses.push_back(std::move(impulse));
}

problem_t make_problem(std::string const& name, int count) {
  problem_t problem;
  problem.friction = name == "sliding" ? 0.6 : 0.0;
  for (int i = 0; i < count; ++i) {
    EntityId const id {static_cast<std::uint64_t>(i + 1)};
    Scalar const mass = name == "mass_ratio" ? (i % 2 == 0 ? 0.1 : 10.0) : 1.0;
    constraint_node_t node;
    node.mass *= mass;
    node.velocity[2] = -9.81 * 0.005;
    if (name == "separating") {
      node.velocity[2] = 1.0;
    } else if (name == "sliding") {
      node.velocity[0] = 0.1;
    }
    node.momentum = node.mass * node.velocity;
    Vector6 expected = Vector6::Zero();
    if (name == "separating") {
      expected = node.velocity;
    } else if (name == "sliding") {
      // The simulation projector preserves the positive normal and clamps
      // tangents radially (it is not a Euclidean SOCP projection). Sliding
      // therefore cancels normal velocity and applies tangential impulse mu*n.
      Scalar const mu = problem.friction;
      Scalar const normal = 9.81 * 0.005;
      expected[0] = 0.1 - mu * normal;
      expected[2] = -9.81 * 0.005 + normal;
    }
    problem.expected.emplace(id, expected);
    problem.nodes.emplace(id, std::move(node));
    if (i == 0) {
      add_contact(problem, {id});
    } else {
      add_contact(problem, {EntityId {static_cast<std::uint64_t>(i)}, id});
    }
  }
  // A vertical frictionless chain with downward free velocities has the
  // unique optimum v=0 for every mass; this holds for heterogeneous masses.
  return problem;
}

struct error_t {
  Scalar momentum = 0.0;
  Scalar cone = 0.0;
  Scalar velocity = 0.0;
};

error_t measure(problem_t const& problem, simulation_solver_config_t const& config) {
  std::map<EntityId, Vector6> residual;
  error_t out;
  for (auto const& [id, node] : problem.nodes) {
    residual[id] = node.mass * node.velocity - node.momentum;
    out.velocity = std::max(out.velocity,
      (node.velocity - problem.expected.at(id)).cwiseAbs().maxCoeff());
  }
  for (std::size_t i = 0; i < problem.factors.size(); ++i) {
    auto const& factor = problem.factors[i];
    // Uniform rows in these fixtures: recover the unscaled multiplier from
    // the projector output; stationarity is checked independently of stats.
    Scalar const scale = std::sqrt(
      config.damping / static_cast<Scalar>(factor.entities.size())) / 0.005;
    Vector3 const lambda = scale * *problem.impulses[i];
    Vector3 g = factor.error;
    for (std::size_t j = 0; j < factor.entities.size(); ++j) {
      auto const id = factor.entities[j];
      auto const& J = factor.jacobians[j];
      g += J * problem.nodes.at(id).velocity;
      residual[id] -= J.transpose() * lambda;
    }
    // Independent contact projection fixed-point diagnostic, for benchmarking
    // only. This retains the production radial Coulomb law. The production
    // stopping rule remains the impulse-change residual.
    out.cone = std::max(out.cone,
      (lambda - project_coulomb_impulse(lambda - g, problem.friction))
        .cwiseAbs().maxCoeff());
  }
  for (auto const& [id, r] : residual) {
    (void)id;
    out.momentum = std::max(out.momentum, r.cwiseAbs().maxCoeff());
  }
  return out;
}

void run(
  std::string const& name, int count, bool adaptive, Scalar beta, bool warm,
  Scalar tol) {
  simulation_solver_config_t config;
  config.tol_abs = tol;
  config.tol_rel = tol * 0.1;
  config.beta_init = beta;
  if (!adaptive) {
    config.beta_min = config.beta_max = beta;
  }
  config.stagnation_window = 0;  // Equal accuracy target and 2000-sweep cap.
  simulation_solver_stats_t stats;
  error_t error;
  std::array<double, 5> times;
  for (double& time : times) {
    problem_t problem = make_problem(name, count);
    ConstraintSolver solver;
    for (auto const& factor : problem.factors) {
      solver.add_factor(factor);
    }
    if (warm) {
      (void)solver.solve(problem.nodes, config, 1.0);
    }
    auto const start = std::chrono::steady_clock::now();
    stats = solver.solve(problem.nodes, config, 1.0);
    time = std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - start).count();
    error = measure(problem, config);
  }
  std::sort(times.begin(), times.end());
  std::cout << tol << ',' << name << ',' << count << ',' << (warm ? "warm" : "cold") << ','
    << (adaptive ? "adaptive" : "fixed") << ',' << beta << ','
    << stats.iters << ',' << stats.converged << ','
    << stats.primal_residual << ',' << stats.dual_residual << ','
    << error.momentum << ',' << error.cone << ',' << error.velocity << ','
    << times[times.size() / 2] << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  Scalar const tol = argc > 1 ? std::stod(argv[1]) : 1e-8;
  std::cout << std::setprecision(9)
    << "tol_abs,scene,bodies,start,mode,beta_init,iters,converged,primal,dual,"
       "momentum_error,cone_error,velocity_error,median_us\n";
  for (std::string const name : {"separating", "sliding", "chain", "mass_ratio"}) {
    for (int count : {1, 2, 4, 8, 16}) {
      if (name != "chain" && name != "mass_ratio" && count != 1) {
        continue;
      }
      for (bool warm : {false, true}) {
        for (bool adaptive : {false, true}) {
          for (Scalar beta : {0.01, 0.1, 1.0, 10.0, 100.0}) {
            run(name, count, adaptive, beta, warm, tol);
          }
        }
      }
    }
  }
}
