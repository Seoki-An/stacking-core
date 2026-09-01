#pragma once

#include "force_graph.hpp"

#include <stacking_core/collision.hpp>

#include <map>
#include <memory>
#include <set>
#include <vector>

namespace stacking_core::posegen_detail {

struct objective_eval_t {
  Scalar objective = 0.0;
  Vector6 grad = Vector6::Zero();
  Matrix6 hess = Matrix6::Zero();
  Scalar c_feq = 0.0;
  Scalar c_comp = 0.0;
  Scalar c_gap = 0.0;
  std::map<EntityId, Vector6> net_wrench;
  std::map<EntityId, std::vector<Vector3>> contact_force;
  std::map<EntityId, std::vector<Vector3>> contact_point;
  std::map<EntityId, std::vector<Vector3>> contact_normal;
  posegen_force_solver_stats_t force_solver;
  int scene_graph_rebuilds = 0;
  int scene_graph_reuses = 0;
};

class PoseObjective {
public:
  PoseObjective(
    posegen_config_t const& config,
    posegen_problem_t const& problem);

  [[nodiscard]] bool can_reuse(posegen_problem_t const& problem) const;
  void begin_solve(posegen_problem_t const& problem);
  [[nodiscard]] objective_eval_t eval(pose_t const& candidate_pose);

private:
  struct ground_contact_t {
    Scalar gap = 0.0;
    Scalar friction = 0.0;
    Vector3 point = Vector3::Zero();
    Vector3 normal = Vector3::UnitZ();
  };

  struct pair_contact_t {
    Scalar gap = 0.0;
    Scalar friction = 0.0;
    Vector3 point_first = Vector3::Zero();
    Vector3 point_second = Vector3::Zero();
    Vector3 normal = Vector3::UnitZ();
  };

  struct force_result_t {
    Scalar c_feq = 0.0;
    Scalar c_comp = 0.0;
    std::map<EntityId, Vector6> net_wrench;
    posegen_force_solver_stats_t solver;
  };

  struct hausdorff_result_t {
    Scalar gap = 0.0;
    Vector3 point_candidate = Vector3::Zero();
    Vector3 point_target = Vector3::Zero();
    Vector3 normal = Vector3::UnitZ();
    Eigen::Matrix<Scalar, 1, 6> d_gap =
      Eigen::Matrix<Scalar, 1, 6>::Zero();
  };

  [[nodiscard]] std::shared_ptr<SceneSnapshot const> make_scene(
    pose_t const& candidate_pose) const;
  void cache_scene_contacts();
  void compute_scene_components();
  [[nodiscard]] force_result_t solve_force(
    std::shared_ptr<SceneSnapshot const> const& scene,
    pose_t const& candidate_pose);
  [[nodiscard]] hausdorff_result_t solve_hausdorff(
    BodyInstance const& target,
    BodyInstance const& candidate) const;

  posegen_config_t const& config_;
  posegen_problem_t problem_;
  pose_t initial_pose_;
  narrow_phase_config_t narrow_config_;
  std::set<EntityId> target_ids_;
  std::set<EntityId> boundary_ids_;
  std::vector<EntityId> scene_ids_;
  std::map<EntityId, std::vector<ground_contact_t>> scene_ground_contacts_;
  std::map<std::pair<EntityId, EntityId>, std::vector<pair_contact_t>>
    scene_pair_contacts_;
  std::map<EntityId, EntityId> scene_component_;
  ForceGraph graph_;
  bool scene_graph_ready_ = false;
  std::set<EntityId> graph_reachable_;
  int scene_graph_rebuilds_ = 0;
  int scene_graph_reuses_ = 0;
};

}  // namespace stacking_core::posegen_detail
