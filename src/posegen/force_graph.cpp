#include "force_graph.hpp"

#include "cone_qp.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

namespace stacking_core::posegen_detail {
namespace {

Vector4 project_cone(Vector4 const& f, Scalar friction) {
  Scalar const r = f.head<3>().norm();
  Scalar n = f[3];
  if (n < -friction * r) {
    return Vector4::Zero();
  }
  if (r < friction * n) {
    return f;
  }
  n = (friction * r + n) / (friction * friction + 1.0);
  Vector4 out;
  out << friction * n * f.head<3>().normalized(), n;
  return out;
}

// Gradient of the smoothed friction cone that project_cone() enforces,
// sqrt(f_0^2 + f_1^2 + eps^2) - friction * f_2, in contact coordinates
// ordered (tangent_first, tangent_second, normal). The denominator is the
// smoothed tangential magnitude, so it must not mix in the normal component.
Vector3 cone_grad(Vector3 const& f, Scalar friction, Scalar eps) {
  Vector3 out;
  out << f.head<2>() / Vector3 {f[0], f[1], eps}.norm(), -friction;
  return out;
}

Vector4 expand_force(Vector3 const& force, Scalar eps) {
  return Vector4 {force[0], force[1], eps, force[2]};
}

Vector3 reduce_force(Vector4 const& force) {
  return Vector3 {force[0], force[1], force[3]};
}

bool valid_mass(Scalar mass) {
  return std::isfinite(mass) && mass > 0.0;
}

}  // namespace

ForceGraph::ForceGraph(
  posegen_config_t const& config, EntityId candidate)
    : config_(config), candidate_(candidate) {
}

void ForceGraph::clear() {
  nodes_.clear();
  edges<force_factor_e::ground_contact>().clear();
  edges<force_factor_e::pair_contact>().clear();
}

void ForceGraph::add_node(EntityId id, Scalar mass, bool boundary) {
  force_graph::node_t& node = nodes_[id];
  node.mass = mass;
  node.raw_mass = mass;
  node.boundary = boundary;
}

template <force_factor_e factor>
force_graph::edge_t<factor>& ForceGraph::add_edge(
  typename force_graph::edge_t<factor>::entity_array_t const& entities,
  typename force_graph::edge_t<factor>::jac_array_t const& jac,
  typename force_graph::edge_t<factor>::point_array_t const& points,
  typename force_graph::edge_t<factor>::point_array_t const& normals,
  Scalar gap,
  Scalar friction,
  Matrix3 const& contact_from_force) {
  auto iter = edges<factor>().emplace(
    entities, force_graph::edge_t<factor> {});
  force_graph::edge_t<factor>& edge = iter->second;
  for (std::size_t idx = 0; idx < edge.cardinality; ++idx) {
    force_graph::node_t& node = nodes_.at(entities[idx]);
    node.lhs_scale_storage.push_back(Vector3::Zero());
    node.rhs_storage.push_back(Vector3::Zero());
    node.force_storage.push_back(Vector3::Zero());
    node.consensus_dual_storage.push_back(Vector3::Zero());
    node.contact_dual_storage.push_back(Vector4::Zero());
    edge.node_indices[idx] = node.lhs_scale_storage.size() - 1;
  }
  edge.jac = jac;
  edge.gap = gap;
  edge.friction = friction;
  edge.representative_force.setZero();
  for (Vector4& auxiliary : edge.auxiliary) {
    auxiliary.setZero();
  }
  edge.contact_multiplier = 0.0;
  edge.contact_point = points;
  edge.contact_normal = normals;
  edge.contact_from_force = contact_from_force;
  return edge;
}

template <force_factor_e factor>
void ForceGraph::remove_edges(
  typename force_graph::edge_t<factor>::entity_array_t const& entities) {
  while (true) {
    auto iter = edges<factor>().find(entities);
    if (iter == edges<factor>().end()) {
      return;
    }
    auto const removed_indices = iter->second.node_indices;
    edges<factor>().erase(iter);
    for (std::size_t idx = 0; idx < removed_indices.size(); ++idx) {
      EntityId const id = entities[idx];
      std::size_t const removed = removed_indices[idx];
      force_graph::node_t& node = nodes_.at(id);
      node.lhs_scale_storage.erase(node.lhs_scale_storage.begin() + removed);
      node.rhs_storage.erase(node.rhs_storage.begin() + removed);
      node.force_storage.erase(node.force_storage.begin() + removed);
      node.consensus_dual_storage.erase(
        node.consensus_dual_storage.begin() + removed);
      node.contact_dual_storage.erase(
        node.contact_dual_storage.begin() + removed);
      for_each_factor([&]<force_factor_e other>() {
        for (auto& [other_ids, edge] : edges<other>()) {
          for (std::size_t other_idx = 0;
               other_idx < edge.cardinality;
               ++other_idx) {
            if (other_ids[other_idx] == id &&
                edge.node_indices[other_idx] > removed) {
              --edge.node_indices[other_idx];
            }
          }
        }
      });
    }
  }
}

void ForceGraph::init() {
  Scalar total_mass = 0.0;
  int mass_count = 0;
  for (auto const& [id, node] : nodes_) {
    static_cast<void>(id);
    if (valid_mass(node.raw_mass)) {
      total_mass += node.raw_mass;
      ++mass_count;
    }
  }
  Scalar const mass_scale =
    mass_count > 0 && total_mass > 0.0 ? total_mass / mass_count : 1.0;

  for (auto& [id, node] : nodes_) {
    static_cast<void>(id);
    node.mass = valid_mass(node.raw_mass)
      ? node.raw_mass / mass_scale
      : node.raw_mass;
    node.jac.resize(0, Eigen::NoChange);
    node.gap.resize(0);
    std::size_t const count = node.lhs_scale_storage.size();
    if (count == 0) {
      node.lhs_scale.reset();
      node.rhs.reset();
      node.force.reset();
      node.consensus_dual.reset();
      node.contact_dual.reset();
      continue;
    }
    node.lhs_scale = std::make_unique<force_graph::node_t::vector_map_t>(
      node.lhs_scale_storage.front().data(), 3 * count);
    node.rhs = std::make_unique<force_graph::node_t::vector_map_t>(
      node.rhs_storage.front().data(), 3 * count);
    node.force = std::make_unique<force_graph::node_t::vector_map_t>(
      node.force_storage.front().data(), 3 * count);
    node.consensus_dual =
      std::make_unique<force_graph::node_t::vector_map_t>(
        node.consensus_dual_storage.front().data(), 3 * count);
    node.contact_dual =
      std::make_unique<force_graph::node_t::vector_map_t>(
        node.contact_dual_storage.front().data(), 4 * count);
  }

  for_each_factor([&]<force_factor_e factor>() {
    for (auto& [entities, edge] : edges<factor>()) {
      bool const scene_edge = std::none_of(
        entities.begin(), entities.end(),
        [&](EntityId id) { return id == candidate_; });
      for (std::size_t idx = 0; idx < edge.cardinality; ++idx) {
        force_graph::node_t& node = nodes_.at(entities[idx]);
        std::size_t const node_idx = edge.node_indices[idx];
        Eigen::Index const rows = static_cast<Eigen::Index>(3 * (node_idx + 1));
        if (node.jac.rows() < rows) {
          node.jac.conservativeResize(rows, Eigen::NoChange);
          node.gap.conservativeResize(rows);
        }
        node.jac.middleRows<3>(3 * node_idx) = edge.jac[idx];
        if (scene_edge) {
          node.gap.segment<3>(3 * node_idx).setZero();
        } else {
          Scalar const gap = std::max(
            edge.gap - config_.objective.eps_comp, Scalar {0.0});
          node.gap.segment<3>(3 * node_idx).setConstant(gap * gap);
        }
        edge.force[idx] = &node.force_storage[node_idx];
        edge.consensus_dual[idx] =
          &node.consensus_dual_storage[node_idx];
        edge.contact_dual[idx] = &node.contact_dual_storage[node_idx];
        edge.lhs_scale[idx] = &node.lhs_scale_storage[node_idx];
        edge.rhs[idx] = &node.rhs_storage[node_idx];
      }
    }
  });
}

posegen_force_solver_stats_t ForceGraph::solve() {
  init();
  posegen_objective_config_t const& obj = config_.objective;
  posegen_force_solver_stats_t stats;

  if (obj.k_wrench.cwiseAbs().maxCoeff() == 0.0 && obj.k_comp == 0.0) {
    for (auto& [id, node] : nodes_) {
      static_cast<void>(id);
      if (node.force) {
        node.force->setZero();
        node.consensus_dual->setZero();
        node.contact_dual->setZero();
      }
    }
    for_each_factor([&]<force_factor_e factor>() {
      for (auto& [entities, edge] : edges<factor>()) {
        static_cast<void>(entities);
        edge.representative_force.setZero();
        for (Vector4& auxiliary : edge.auxiliary) {
          auxiliary.setZero();
        }
        edge.contact_multiplier = 0.0;
      }
    });
    return stats;
  }

  Matrix6 const zero_wrench = Matrix6::Zero();
  auto node_wrench = [&](force_graph::node_t const& node) -> Matrix6 const& {
    return node.boundary ? zero_wrench : obj.k_wrench;
  };
  Vector6 gravity;
  gravity << 0.0, 0.0, -obj.gravity, 0.0, 0.0, 0.0;
  for (auto& [id, node] : nodes_) {
    if (node.jac.rows() == 0) {
      continue;
    }
    node.lhs_diag_base.setConstant(node.jac.rows(), obj.rho);
    if (id == candidate_) {
      node.lhs_diag_base.array() += obj.k_comp * node.gap.array();
    }
    node.q = node.jac * node_wrench(node) * node.mass * gravity;
  }

  stats = config_.force_solver.method ==
      posegen_force_solver_e::interior_point
    ? solve_interior_point()
    : solve_graph_admm();

  for_each_factor([&]<force_factor_e factor>() {
    for (auto& [entities, edge] : edges<factor>()) {
      Vector3 force_diff = Vector3::Zero();
      for (std::size_t idx = 0; idx < edge.cardinality; ++idx) {
        force_graph::node_t const& node = nodes_.at(entities[idx]);
        if (node.boundary) {
          continue;
        }
        force_diff += edge.jac[idx] * obj.k_wrench *
          (node.jac.transpose() * *node.force + node.mass * gravity);
      }
      Scalar const gap = std::max(edge.gap - obj.eps_comp, Scalar {0.0});
      force_diff +=
        (obj.rho + obj.k_comp * gap * gap) * edge.representative_force;
      Vector3 const grad = cone_grad(
        edge.representative_force, edge.friction, obj.eps_cone);
      edge.contact_multiplier = -grad.dot(force_diff) / grad.squaredNorm();
    }
  });
  return stats;
}


posegen_force_solver_stats_t ForceGraph::solve_graph_admm() {
  posegen_objective_config_t const& obj = config_.objective;
  posegen_force_solver_config_t const& solver = config_.force_solver;
  posegen_force_solver_stats_t stats;
  Matrix6 const zero_wrench = Matrix6::Zero();
  auto node_wrench = [&](force_graph::node_t const& node) -> Matrix6 const& {
    return node.boundary ? zero_wrench : obj.k_wrench;
  };

  Scalar beta_contact = solver.beta_contact;
  Scalar beta_consensus = solver.beta_consensus;
  bool lhs_dirty = true;
  stats.converged = false;
  for (int iter = 0; iter < solver.max_iters; ++iter) {
    Scalar dual_res = 0.0;
    Scalar primal_res = 0.0;
    Scalar primal_scale = 1e-3;
    Scalar dual_scale = 1e-3;

    for (auto& [entities, edge] :
         edges<force_factor_e::ground_contact>()) {
      static_cast<void>(entities);
      edge.representative_force = *edge.force[0];
      *edge.rhs[0] = reduce_force(
        beta_contact * edge.auxiliary[0] - *edge.contact_dual[0]);
      edge.lhs_scale[0]->setConstant(beta_contact);
    }
    for (auto& [entities, edge] : edges<force_factor_e::pair_contact>()) {
      static_cast<void>(entities);
      edge.representative_force =
        (*edge.force[0] + *edge.consensus_dual[0] / beta_consensus +
         *edge.force[1] + *edge.consensus_dual[1] / beta_consensus) /
        2.0;
      for (std::size_t idx = 0; idx < 2; ++idx) {
        *edge.rhs[idx] = reduce_force(
          beta_contact * edge.auxiliary[idx] - *edge.contact_dual[idx]);
        edge.lhs_scale[idx]->setConstant(beta_contact);
        *edge.rhs[idx] += beta_consensus * edge.representative_force -
          *edge.consensus_dual[idx];
        edge.lhs_scale[idx]->array() += beta_consensus;
      }
    }

    for (auto& [id, node] : nodes_) {
      static_cast<void>(id);
      if (node.jac.rows() == 0) {
        continue;
      }
      Matrix6 const& k_wrench = node_wrench(node);
      if (lhs_dirty) {
        node.d_inv =
          (node.lhs_diag_base.array() + node.lhs_scale->array()).inverse();
        node.d_inv_jac = node.d_inv.asDiagonal() * node.jac;
        Matrix6 capacitance = Matrix6::Identity();
        capacitance.noalias() +=
          k_wrench * (node.jac.transpose() * node.d_inv_jac);
        node.capacitance_lu.compute(capacitance);
      }
      *node.rhs -= node.q;
      Vector6 const proj = node.capacitance_lu.solve(
        k_wrench * (node.d_inv_jac.transpose() * *node.rhs));
      *node.force = node.d_inv.array() * node.rhs->array();
      node.force->noalias() -= node.d_inv_jac * proj;

      node.dual_res = *node.rhs;
      node.dual_res.array() -=
        node.lhs_scale->array() * node.force->array();
      dual_scale = std::max(
        dual_scale,
        std::max(
          node.dual_res.cwiseAbs().maxCoeff(),
          node.q.cwiseAbs().maxCoeff()));
      node.dual_res += node.q;
      node.dual_res += *node.consensus_dual;
      Eigen::VectorXd const& contact_dual = *node.contact_dual;
      for (Eigen::Index idx = 0; 3 * idx < node.dual_res.size(); ++idx) {
        node.dual_res[3 * idx] += contact_dual[4 * idx];
        node.dual_res[3 * idx + 1] += contact_dual[4 * idx + 1];
        node.dual_res[3 * idx + 2] += contact_dual[4 * idx + 3];
      }
      dual_res += node.dual_res.norm();
    }
    lhs_dirty = false;

    for (auto& [entities, edge] :
         edges<force_factor_e::ground_contact>()) {
      static_cast<void>(entities);
      Vector4 const expanded = expand_force(*edge.force[0], obj.eps_cone);
      edge.auxiliary[0] =
        expanded + *edge.contact_dual[0] / beta_contact;
      edge.auxiliary[0] = project_cone(edge.auxiliary[0], edge.friction);
      *edge.contact_dual[0] +=
        beta_contact * (expanded - edge.auxiliary[0]);

      primal_res += (*edge.force[0] - edge.representative_force).norm();
      primal_res += (expanded - edge.auxiliary[0]).norm();
      primal_scale = std::max(
        primal_scale, edge.force[0]->cwiseAbs().maxCoeff());
      primal_scale = std::max(
        primal_scale, edge.auxiliary[0].cwiseAbs().maxCoeff());
      dual_scale = std::max(
        dual_scale,
        std::max(
          edge.consensus_dual[0]->cwiseAbs().maxCoeff(),
          edge.contact_dual[0]->cwiseAbs().maxCoeff()));
    }
    for (auto& [entities, edge] : edges<force_factor_e::pair_contact>()) {
      static_cast<void>(entities);
      for (std::size_t idx = 0; idx < 2; ++idx) {
        Vector4 const expanded = expand_force(*edge.force[idx], obj.eps_cone);
        edge.auxiliary[idx] =
          expanded + *edge.contact_dual[idx] / beta_contact;
        edge.auxiliary[idx] = project_cone(
          edge.auxiliary[idx], edge.friction);
        *edge.consensus_dual[idx] +=
          beta_consensus * (*edge.force[idx] - edge.representative_force);
        *edge.contact_dual[idx] +=
          beta_contact * (expanded - edge.auxiliary[idx]);

        primal_res +=
          (*edge.force[idx] - edge.representative_force).norm();
        primal_res += (expanded - edge.auxiliary[idx]).norm();
        primal_scale = std::max(
          primal_scale, edge.force[idx]->cwiseAbs().maxCoeff());
        primal_scale = std::max(
          primal_scale, edge.auxiliary[idx].cwiseAbs().maxCoeff());
        dual_scale = std::max(
          dual_scale,
          std::max(
            edge.consensus_dual[idx]->cwiseAbs().maxCoeff(),
            edge.contact_dual[idx]->cwiseAbs().maxCoeff()));
      }
    }

    if ((iter + 1) % solver.beta_update_interval == 0) {
      Scalar const primal_ratio =
        primal_scale > 0.0 ? primal_res / primal_scale : 0.0;
      Scalar const dual_ratio =
        dual_scale > 0.0 ? dual_res / dual_scale : 0.0;
      Scalar const beta_ratio = dual_ratio > 0.0
        ? std::sqrt(primal_ratio / dual_ratio)
        : 1.0;
      if (std::isfinite(beta_ratio) && beta_ratio > 0.0) {
        beta_consensus *= beta_ratio;
        beta_contact *= beta_ratio;
        lhs_dirty = true;
      }
    }
    Scalar const primal_tol =
      solver.tol_abs + solver.tol_rel * primal_scale;
    Scalar const dual_tol = solver.tol_abs + solver.tol_rel * dual_scale;
    stats.iters = iter + 1;
    stats.primal_residual = primal_res;
    stats.dual_residual = dual_res;
    if (primal_res < primal_tol && dual_res < dual_tol) {
      stats.converged = true;
      break;
    }
  }

  return stats;
}

// Solves the same subproblem directly over one force per contact. Node forces
// are written back afterwards so every downstream consumer -- the cone KKT
// multiplier below, the reported contact forces, the pose gradient -- sees the
// layout it expects and cannot tell which solver produced it.
posegen_force_solver_stats_t ForceGraph::solve_interior_point() {
  posegen_objective_config_t const& obj = config_.objective;
  Matrix6 const zero_wrench = Matrix6::Zero();
  auto node_wrench = [&](force_graph::node_t const& node) -> Matrix6 const& {
    return node.boundary ? zero_wrench : obj.k_wrench;
  };

  std::map<std::pair<EntityId, std::size_t>, std::size_t> slot_to_contact;
  cone_qp_problem_t problem;
  problem.eps = obj.eps_cone;
  std::size_t contact_count = 0;
  for_each_factor([&]<force_factor_e factor>() {
    for (auto& [entities, edge] : edges<factor>()) {
      for (std::size_t idx = 0; idx < edge.cardinality; ++idx) {
        slot_to_contact[{entities[idx], edge.node_indices[idx]}] =
          contact_count;
      }
      problem.friction.push_back(edge.friction);
      ++contact_count;
    }
  });

  Eigen::Index const dim = static_cast<Eigen::Index>(3 * contact_count);
  problem.hessian = Eigen::MatrixXd::Zero(dim, dim);
  problem.linear = Eigen::VectorXd::Zero(dim);
  for (auto& [id, node] : nodes_) {
    if (node.jac.rows() == 0) {
      continue;
    }
    Eigen::Index const slots = node.jac.rows() / 3;
    Eigen::MatrixXd block =
      node.jac * node_wrench(node) * node.jac.transpose();
    block.diagonal() += node.lhs_diag_base;
    for (Eigen::Index a = 0; a < slots; ++a) {
      Eigen::Index const row = static_cast<Eigen::Index>(
        3 * slot_to_contact.at({id, static_cast<std::size_t>(a)}));
      problem.linear.segment<3>(row) += node.q.segment<3>(3 * a);
      for (Eigen::Index b = 0; b < slots; ++b) {
        Eigen::Index const col = static_cast<Eigen::Index>(
          3 * slot_to_contact.at({id, static_cast<std::size_t>(b)}));
        problem.hessian.block<3, 3>(row, col) +=
          block.block<3, 3>(3 * a, 3 * b);
      }
    }
  }

  posegen_force_solver_stats_t const stats = solve_cone_qp(
    problem, interior_point_forces_, config_.force_solver);

  for (auto& [id, node] : nodes_) {
    static_cast<void>(id);
    if (node.force) {
      node.force->setZero();
      node.consensus_dual->setZero();
      node.contact_dual->setZero();
    }
  }
  for_each_factor([&]<force_factor_e factor>() {
    for (auto& [entities, edge] : edges<factor>()) {
      Eigen::Index const row = static_cast<Eigen::Index>(
        3 * slot_to_contact.at({entities[0], edge.node_indices[0]}));
      edge.representative_force = interior_point_forces_.segment<3>(row);
      for (std::size_t idx = 0; idx < edge.cardinality; ++idx) {
        *edge.force[idx] = edge.representative_force;
        edge.auxiliary[idx] = Vector4 {
          edge.representative_force[0],
          edge.representative_force[1],
          obj.eps_cone,
          edge.representative_force[2]};
      }
    }
  });
  return stats;
}

template force_graph::edge_t<force_factor_e::ground_contact>&
ForceGraph::add_edge<force_factor_e::ground_contact>(
  force_graph::edge_t<force_factor_e::ground_contact>::entity_array_t const&,
  force_graph::edge_t<force_factor_e::ground_contact>::jac_array_t const&,
  force_graph::edge_t<force_factor_e::ground_contact>::point_array_t const&,
  force_graph::edge_t<force_factor_e::ground_contact>::point_array_t const&,
  Scalar,
  Scalar,
  Matrix3 const&);

template force_graph::edge_t<force_factor_e::pair_contact>&
ForceGraph::add_edge<force_factor_e::pair_contact>(
  force_graph::edge_t<force_factor_e::pair_contact>::entity_array_t const&,
  force_graph::edge_t<force_factor_e::pair_contact>::jac_array_t const&,
  force_graph::edge_t<force_factor_e::pair_contact>::point_array_t const&,
  force_graph::edge_t<force_factor_e::pair_contact>::point_array_t const&,
  Scalar,
  Scalar,
  Matrix3 const&);

template void ForceGraph::remove_edges<force_factor_e::ground_contact>(
  force_graph::edge_t<force_factor_e::ground_contact>::entity_array_t const&);

template void ForceGraph::remove_edges<force_factor_e::pair_contact>(
  force_graph::edge_t<force_factor_e::pair_contact>::entity_array_t const&);

}  // namespace stacking_core::posegen_detail
