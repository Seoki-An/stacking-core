#include "constraint_solver.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace stacking_core::simulation_detail {
namespace {

struct node_data_t {
  Matrix6 lhs = Matrix6::Zero();
  Eigen::LLT<Matrix6> lhs_llt;
  Vector6 rhs = Vector6::Zero();
  Vector6 residual = Vector6::Zero();
  Vector6* var = nullptr;
  bool in_admm = false;
};

struct factor_data_t {
  constraint_factor_t const* factor = nullptr;
  vector_x_t scale;
  vector_x_t scale_inv;
  std::vector<matrix_x6_t> jacobians;
  vector_x_t error;
  std::vector<vector_x_t> auxiliary;
  std::vector<vector_x_t> jx;
  vector_x_t impulse;
  vector_x_t residual;
  vector_x_t scaled_residual;
  std::vector<node_data_t*> nodes;
};

void validate_config(simulation_solver_config_t const& config) {
  if (!std::isfinite(config.damping) || config.damping <= 0.0 ||
      !std::isfinite(config.beta_init) || config.beta_init <= 0.0 ||
      !std::isfinite(config.beta_min) || config.beta_min <= 0.0 ||
      !std::isfinite(config.beta_max) ||
      config.beta_max < config.beta_min ||
      config.beta_update_interval <= 0 || config.max_iters <= 0 ||
      !std::isfinite(config.tol_abs) || config.tol_abs < 0.0 ||
      !std::isfinite(config.tol_rel) || config.tol_rel < 0.0 ||
      config.stagnation_window < 0 ||
      !std::isfinite(config.stagnation_tol) ||
      config.stagnation_tol < 0.0 ||
      !std::isfinite(config.stagnation_min_metric) ||
      config.stagnation_min_metric < 0.0 ||
      !std::isfinite(config.stagnation_headroom) ||
      config.stagnation_headroom < 1.0 ||
      !std::isfinite(config.stagnation_max_metric) ||
      config.stagnation_max_metric < 0.0) {
    throw std::invalid_argument("invalid simulation solver configuration");
  }
}

}  // namespace

void ConstraintSolver::clear_factors() {
  factors_.clear();
}

void ConstraintSolver::clear_warm_start() {
  warm_start_.clear();
  warm_beta_ = -1.0;
}

void ConstraintSolver::add_factor(constraint_factor_t factor) {
  Eigen::Index const dim = factor.error.size();
  if (dim <= 0 || !factor.error.allFinite() || factor.entities.empty() ||
      factor.entities.size() != factor.jacobians.size()) {
    throw std::invalid_argument("invalid simulation constraint factor");
  }
  for (matrix_x6_t const& J : factor.jacobians) {
    if (J.rows() != dim || J.cols() != 6 || !J.allFinite()) {
      throw std::invalid_argument("invalid simulation constraint Jacobian");
    }
  }
  if (factor.hard_inequality && !factor.projector) {
    throw std::invalid_argument("hard constraint requires an impulse projector");
  }
  factors_.push_back(std::move(factor));
}

simulation_solver_stats_t ConstraintSolver::solve(
  std::map<EntityId, Vector6>& vars,
  simulation_solver_config_t const& config,
  Scalar dynamics_scale) {
  validate_config(config);
  if (!std::isfinite(dynamics_scale) || dynamics_scale <= 0.0) {
    throw std::invalid_argument("simulation dynamics scale must be positive");
  }

  Scalar const damping = config.damping * dynamics_scale;
  Scalar beta = warm_beta_ > 0.0 ? warm_beta_ : config.beta_init;
  beta = std::clamp(beta, config.beta_min, config.beta_max);
  Scalar beta_ratio = 1.0;

  std::map<EntityId, node_data_t> nodes;
  for (auto& [id, var] : vars) {
    if (!var.allFinite()) {
      throw std::invalid_argument("simulation motion variable must be finite");
    }
    nodes[id].var = &var;
  }

  std::vector<factor_data_t> data;
  data.reserve(factors_.size());
  for (constraint_factor_t const& factor : factors_) {
    factor_data_t& item = data.emplace_back();
    item.factor = &factor;
    Eigen::Index const dim = factor.error.size();
    vector_x_t scale_norm_sqrt = vector_x_t::Zero(dim);
    item.nodes.reserve(factor.entities.size());
    for (std::size_t i = 0; i < factor.entities.size(); ++i) {
      auto node = nodes.find(factor.entities[i]);
      if (node == nodes.end()) {
        throw std::invalid_argument(
          "constraint factor references a missing motion variable");
      }
      item.nodes.push_back(&node->second);
      scale_norm_sqrt +=
        factor.jacobians[i].rowwise().squaredNorm() / damping;
    }
    if (!factor.hard_inequality) {
      scale_norm_sqrt.array() += 1.0;
    }
    item.scale = scale_norm_sqrt.array().sqrt().inverse();
    if (factor.hard_inequality) {
      item.scale.setConstant(item.scale.mean());
    }
    item.scale_inv = item.scale.cwiseInverse();
    item.error = item.scale.array() * factor.error.array();
    item.jacobians.reserve(factor.jacobians.size());
    for (matrix_x6_t const& J : factor.jacobians) {
      item.jacobians.push_back(item.scale.asDiagonal() * J);
    }

    item.impulse = vector_x_t::Zero(dim);
    item.residual = vector_x_t::Zero(dim);
    item.scaled_residual = vector_x_t::Zero(dim);
    item.auxiliary.assign(factor.entities.size(), vector_x_t::Zero(dim));
    item.jx.assign(factor.entities.size(), vector_x_t::Zero(dim));
    auto const warm = warm_start_.find(factor.id);
    if (warm != warm_start_.end() &&
        warm->second.impulse.size() == dim &&
        warm->second.auxiliary.size() == factor.entities.size()) {
      item.impulse = warm->second.impulse;
      bool valid = item.impulse.allFinite();
      for (std::size_t i = 0; i < item.auxiliary.size(); ++i) {
        valid = valid && warm->second.auxiliary[i].size() == dim &&
          warm->second.auxiliary[i].allFinite();
      }
      if (valid) {
        item.auxiliary = warm->second.auxiliary;
      } else {
        item.impulse.setZero();
        for (vector_x_t& y : item.auxiliary) {
          y.setZero();
        }
      }
    }

    if (factor.hard_inequality || factor.entities.size() > 1) {
      for (node_data_t* node : item.nodes) {
        node->in_admm = true;
      }
    }
  }

  bool any_free = false;
  for (auto& [id, node] : nodes) {
    (void)id;
    if (!node.in_admm) {
      node.lhs.setZero();
      node.lhs.diagonal().setConstant(damping);
      node.rhs.setZero();
      any_free = true;
    }
  }
  if (any_free) {
    for (factor_data_t& item : data) {
      if (item.factor->hard_inequality) {
        continue;
      }
      for (std::size_t i = 0; i < item.nodes.size(); ++i) {
        node_data_t* node = item.nodes[i];
        if (node->in_admm) {
          continue;
        }
        matrix_x6_t const& J = item.factor->jacobians[i];
        node->lhs += J.transpose() * J;
        node->rhs -= J.transpose() * item.factor->error;
      }
    }
    for (auto& [id, node] : nodes) {
      (void)id;
      if (!node.in_admm) {
        node.lhs_llt.compute(node.lhs);
        if (node.lhs_llt.info() != Eigen::Success) {
          throw std::runtime_error("simulation factorization failed");
        }
        *node.var = node.lhs_llt.solve(node.rhs);
      }
    }
    for (factor_data_t& item : data) {
      if (item.factor->hard_inequality) {
        continue;
      }
      for (std::size_t i = 0; i < item.nodes.size(); ++i) {
        node_data_t* node = item.nodes[i];
        if (node->in_admm) {
          continue;
        }
        vector_x_t const jx =
          item.factor->jacobians[i] * *node->var;
        item.impulse =
          (-(jx + item.factor->error).array() * item.scale_inv.array())
            .matrix();
        item.auxiliary[i] =
          (beta * item.scale.array() * jx.array() - item.impulse.array())
            .matrix();
      }
    }
  }

  std::vector<node_data_t*> active_nodes;
  for (auto& [id, node] : nodes) {
    (void)id;
    if (node.in_admm) {
      active_nodes.push_back(&node);
    }
  }
  std::vector<factor_data_t*> active_factors;
  for (factor_data_t& item : data) {
    bool active = false;
    for (node_data_t* node : item.nodes) {
      active = active || node->in_admm;
    }
    if (active) {
      active_factors.push_back(&item);
    }
  }

  simulation_solver_stats_t stats;
  if (active_nodes.empty()) {
    warm_beta_ = beta;
    warm_start_.clear();
    for (factor_data_t const& item : data) {
      warm_start_[item.factor->id] =
        warm_factor_t {.impulse = item.impulse, .auxiliary = item.auxiliary};
    }
    return stats;
  }

  Scalar stagnation_best = std::numeric_limits<Scalar>::infinity();
  int stagnation_last_improve = 0;
  stats.converged = false;
  for (int iter = 0; iter < config.max_iters; ++iter) {
    Scalar primal_var_norm = 0.0;
    Scalar scaled_primal_var_norm = 0.0;
    Scalar dual_var_norm = 0.0;
    Scalar primal_residual = 0.0;
    Scalar scaled_primal_residual = 0.0;

    for (node_data_t* node : active_nodes) {
      node->rhs.setZero();
    }
    for (factor_data_t* item : active_factors) {
      item->residual = item->impulse;
      vector_x_t vec = -beta * item->error;
      for (vector_x_t const& y : item->auxiliary) {
        vec -= y;
      }
      if (item->factor->hard_inequality) {
        item->impulse = item->factor->projector(
          vec / static_cast<Scalar>(item->nodes.size()));
      } else {
        item->impulse =
          (beta * item->scale.array().square() +
           static_cast<Scalar>(item->nodes.size()))
            .inverse() * vec.array();
      }

      for (std::size_t i = 0; i < item->nodes.size(); ++i) {
        item->nodes[i]->rhs += item->jacobians[i].transpose() *
          (beta_ratio * (item->auxiliary[i] + item->impulse) +
           item->impulse);
      }
      item->scaled_residual = (item->residual - item->impulse) / beta;
      item->residual =
        item->scaled_residual.array() * item->scale_inv.array();

      for (std::size_t i = 0; i < item->nodes.size(); ++i) {
        if (iter == 0) {
          item->jx[i] = item->jacobians[i] * *item->nodes[i]->var;
        }
        primal_var_norm = std::max(
          primal_var_norm, item->jx[i].cwiseAbs().maxCoeff());
        scaled_primal_var_norm = std::max(
          scaled_primal_var_norm,
          (item->jx[i].array() * item->scale_inv.array())
            .cwiseAbs().maxCoeff());
      }
      dual_var_norm = std::max(
        dual_var_norm, item->impulse.cwiseAbs().maxCoeff());
      primal_residual = std::max(
        primal_residual, item->residual.cwiseAbs().maxCoeff());
      scaled_primal_residual = std::max(
        scaled_primal_residual,
        item->scaled_residual.cwiseAbs().maxCoeff());
    }
    beta_ratio = 1.0;

    Scalar dual_residual = 0.0;
    for (node_data_t* node : active_nodes) {
      dual_residual = std::max(
        dual_residual, node->residual.cwiseAbs().maxCoeff());
    }
    Scalar const primal_tol =
      config.tol_abs + config.tol_rel * primal_var_norm;
    Scalar const dual_tol =
      config.tol_abs + config.tol_rel * dual_var_norm;
    stats.iters = iter + 1;
    stats.primal_residual = primal_residual;
    stats.dual_residual = dual_residual;
    if (iter > 0 &&
        primal_residual < primal_tol && dual_residual < dual_tol) {
      stats.converged = true;
      break;
    }

    if (config.stagnation_window > 0) {
      Scalar const metric = std::max(
        primal_residual / primal_tol, dual_residual / dual_tol);
      if (metric < stagnation_best * (1.0 - config.stagnation_tol)) {
        stagnation_best = metric;
        stagnation_last_improve = iter;
      } else if (
        iter - stagnation_last_improve >= config.stagnation_window &&
        metric > config.stagnation_min_metric &&
        metric <= config.stagnation_headroom * stagnation_best &&
        metric <= config.stagnation_max_metric) {
        break;
      }
    }

    if (iter % config.beta_update_interval == 0) {
      if (iter > 0) {
        Scalar const primal_ratio = scaled_primal_var_norm > 0.0
          ? scaled_primal_residual / scaled_primal_var_norm
          : 0.0;
        Scalar const dual_ratio = dual_var_norm > 0.0
          ? dual_residual / dual_var_norm
          : 0.0;
        Scalar const candidate = dual_ratio > 0.0
          ? std::sqrt(primal_ratio / dual_ratio)
          : 1.0;
        if (std::isfinite(candidate) && candidate > 0.0) {
          Scalar const beta_new = std::clamp(
            beta * candidate, config.beta_min, config.beta_max);
          beta_ratio = beta_new / beta;
          beta = beta_new;
        }
      }

      if (iter == 0) {
        for (node_data_t* node : active_nodes) {
          node->lhs.setZero();
          node->lhs.diagonal().setConstant(damping);
        }
        for (factor_data_t* item : active_factors) {
          for (std::size_t i = 0; i < item->nodes.size(); ++i) {
            item->nodes[i]->lhs += beta *
              item->jacobians[i].transpose() * item->jacobians[i];
          }
        }
        for (node_data_t* node : active_nodes) {
          node->lhs_llt.compute(node->lhs);
          if (node->lhs_llt.info() != Eigen::Success) {
            throw std::runtime_error("simulation factorization failed");
          }
        }
      } else if (beta_ratio != 1.0) {
        for (node_data_t* node : active_nodes) {
          node->lhs *= beta_ratio;
          node->lhs.diagonal().array() +=
            (1.0 - beta_ratio) * damping;
          node->lhs_llt.compute(node->lhs);
          if (node->lhs_llt.info() != Eigen::Success) {
            throw std::runtime_error("simulation factorization failed");
          }
        }
      }
    }

    for (node_data_t* node : active_nodes) {
      *node->var = node->lhs_llt.solve(node->rhs);
      node->residual = node->rhs;
    }
    for (factor_data_t* item : active_factors) {
      for (std::size_t i = 0; i < item->nodes.size(); ++i) {
        item->jx[i] = item->jacobians[i] * *item->nodes[i]->var;
        item->auxiliary[i] = beta * item->jx[i] - item->impulse;
        item->nodes[i]->residual -=
          item->jacobians[i].transpose() * item->impulse;
      }
    }
  }

  warm_beta_ = beta;
  warm_start_.clear();
  for (factor_data_t const& item : data) {
    warm_start_[item.factor->id] =
      warm_factor_t {.impulse = item.impulse, .auxiliary = item.auxiliary};
  }
  return stats;
}

}  // namespace stacking_core::simulation_detail
