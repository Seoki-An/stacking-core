#pragma once

#include <stacking_core/posegen/types.hpp>

#include <Eigen/LU>

#include <array>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

namespace stacking_core::posegen_detail {

enum class force_factor_e {
  ground_contact,
  pair_contact,
};

template <force_factor_e>
struct force_factor_info_t;

template <>
struct force_factor_info_t<force_factor_e::ground_contact> {
  static constexpr std::size_t cardinality = 1;
};

template <>
struct force_factor_info_t<force_factor_e::pair_contact> {
  static constexpr std::size_t cardinality = 2;
};

namespace force_graph {

struct node_t {
  using vector_map_t = Eigen::Map<Eigen::VectorXd>;

  Scalar mass = 1.0;
  Scalar raw_mass = 1.0;
  bool boundary = false;

  Eigen::Matrix<Scalar, Eigen::Dynamic, 6> jac;
  Eigen::VectorXd gap;
  Eigen::VectorXd q;
  Eigen::VectorXd lhs_diag_base;
  Eigen::VectorXd d_inv;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 6> d_inv_jac;
  Eigen::PartialPivLU<Matrix6> capacitance_lu;
  Eigen::VectorXd dual_res;

  std::vector<Vector3> lhs_scale_storage;
  std::vector<Vector3> rhs_storage;
  std::vector<Vector3> force_storage;
  std::vector<Vector3> consensus_dual_storage;
  std::vector<Vector4> contact_dual_storage;

  std::unique_ptr<vector_map_t> lhs_scale;
  std::unique_ptr<vector_map_t> rhs;
  std::unique_ptr<vector_map_t> force;
  std::unique_ptr<vector_map_t> consensus_dual;
  std::unique_ptr<vector_map_t> contact_dual;
};

template <force_factor_e factor>
struct edge_t {
  static constexpr std::size_t cardinality =
    force_factor_info_t<factor>::cardinality;

  using entity_array_t = std::array<EntityId, cardinality>;
  using jac_array_t = std::array<Matrix36, cardinality>;
  using point_array_t = std::array<Vector3, cardinality>;
  using force_array_t = std::array<Vector3*, cardinality>;
  using dual_array_t = std::array<Vector3*, cardinality>;
  using contact_dual_array_t = std::array<Vector4*, cardinality>;
  using deriv_array_t =
    std::array<std::vector<Matrix36>, cardinality>;

  jac_array_t jac;
  Scalar gap = 0.0;
  Scalar friction = 0.0;

  force_array_t force {};
  Vector3 representative_force = Vector3::Zero();
  std::array<Vector4, cardinality> auxiliary;
  dual_array_t consensus_dual {};
  contact_dual_array_t contact_dual {};
  dual_array_t lhs_scale {};
  dual_array_t rhs {};
  Scalar contact_multiplier = 0.0;
  std::array<std::size_t, cardinality> node_indices {};

  deriv_array_t d_jac;
  Vector6 d_gap = Vector6::Zero();
  Matrix36 d_normal = Matrix36::Zero();
  Matrix3 contact_from_force = Matrix3::Identity();
  point_array_t contact_point;
  point_array_t contact_normal;
};

template <force_factor_e factor>
using edge_map_t = std::multimap<
  typename edge_t<factor>::entity_array_t, edge_t<factor>>;

}  // namespace force_graph

class ForceGraph {
public:
  ForceGraph(posegen_config_t const& config, EntityId candidate);

  void clear();
  void add_node(EntityId id, Scalar mass, bool boundary = false);

  template <force_factor_e factor>
  force_graph::edge_t<factor>& add_edge(
    typename force_graph::edge_t<factor>::entity_array_t const& entities,
    typename force_graph::edge_t<factor>::jac_array_t const& jac,
    typename force_graph::edge_t<factor>::point_array_t const& points,
    typename force_graph::edge_t<factor>::point_array_t const& normals,
    Scalar gap,
    Scalar friction,
    Matrix3 const& contact_from_force);

  template <force_factor_e factor>
  void remove_edges(
    typename force_graph::edge_t<factor>::entity_array_t const& entities);

  template <force_factor_e factor>
  force_graph::edge_map_t<factor>& edges() {
    return std::get<force_graph::edge_map_t<factor>>(edges_);
  }

  template <force_factor_e factor>
  force_graph::edge_map_t<factor> const& edges() const {
    return std::get<force_graph::edge_map_t<factor>>(edges_);
  }

  std::map<EntityId, force_graph::node_t>& nodes() noexcept {
    return nodes_;
  }

  std::map<EntityId, force_graph::node_t> const& nodes() const noexcept {
    return nodes_;
  }

  template <typename Func>
  void for_each_factor(Func&& func) {
    func.template operator()<force_factor_e::ground_contact>();
    func.template operator()<force_factor_e::pair_contact>();
  }

  [[nodiscard]] posegen_force_solver_stats_t solve();

private:
  void init();
  [[nodiscard]] posegen_force_solver_stats_t solve_graph_admm();
  [[nodiscard]] posegen_force_solver_stats_t solve_interior_point();

  posegen_config_t const& config_;
  EntityId candidate_;
  std::map<EntityId, force_graph::node_t> nodes_;
  std::tuple<
    force_graph::edge_map_t<force_factor_e::ground_contact>,
    force_graph::edge_map_t<force_factor_e::pair_contact>> edges_;
  Eigen::VectorXd interior_point_forces_;
};

}  // namespace stacking_core::posegen_detail
