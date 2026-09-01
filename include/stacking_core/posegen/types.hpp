#pragma once

#include <stacking_core/scene/view.hpp>

#include <map>
#include <vector>

namespace stacking_core {

struct posegen_trust_region_config_t {
  int max_iters = 100;
  Scalar eps = 0.1;
  Scalar delta_init = 0.125;
  Scalar delta_max = 1.0;
  Scalar delta_reduction_rate = 0.25;
  Scalar delta_expansion_rate = 2.0;
  Scalar delta_lower_thresh = 0.25;
  Scalar delta_upper_thresh = 0.75;
  Scalar improvement_thresh = 0.125;
  Scalar tol = 1e-8;
};

struct posegen_hausdorff_config_t {
  Scalar eps = 1e-3;
  int max_iters = 50;
  Scalar error_tol = 1e-10;
  Scalar radius_init = 10.0;
  Scalar radius_reduction_rate = 0.25;
  Scalar radius_expansion_rate = 3.0;
  Scalar gain_ratio_lower_thresh = 0.05;
  Scalar gain_ratio_upper_thresh = 0.9;
  Scalar gain_ratio_max = 1e10;
};

struct posegen_objective_config_t {
  Scalar rho = 1e-2;
  Scalar narrow_phase_scene_tol = 5e-3;
  Scalar narrow_phase_candidate_tol = 0.10;
  Scalar eps_gap = 2e-2;
  Scalar eps_comp = 2e-2;
  Scalar eps_cone = 1e-2;
  Scalar eps_target = 5e-1;
  Scalar k_comp = 0.0;
  Matrix6 k_wrench = Matrix6::Identity();
  Scalar k_gap = 0.0;
  Scalar k_gap_c = 80.0;
  Scalar k_target = 1.0;
  Scalar k_potential = 1.0;
  Scalar k_xy = 0.0;
  Scalar k_box = 0.1;
  Scalar k_reg = 0.0;
  Scalar k_lower = 0.0;
  Scalar w_box = 1.0;
  Scalar gravity = 9.81;
  Scalar ground_height = 0.0;
};

struct posegen_force_solver_config_t {
  int max_iters = 1000;
  Scalar beta_consensus = 1.0;
  Scalar beta_contact = 1.0;
  int beta_update_interval = 20;
  Scalar tol_abs = 1e-3;
  Scalar tol_rel = 1e-4;
};

struct posegen_config_t {
  posegen_trust_region_config_t trust_region;
  posegen_hausdorff_config_t hausdorff;
  posegen_objective_config_t objective;
  posegen_force_solver_config_t force_solver;
};

// Candidate, target, and boundary are roles of this solve. Every referenced
// body belongs to the same canonical scene and therefore shares one frame.
struct posegen_problem_t {
  SceneView scene;
  EntityId candidate;
  std::vector<EntityId> targets;
  std::vector<EntityId> boundaries;
};

struct posegen_force_solver_stats_t {
  int iters = 0;
  bool converged = true;
  Scalar primal_residual = 0.0;
  Scalar dual_residual = 0.0;
};

struct posegen_solver_stats_t {
  int iters = 0;
  int accepted_iters = 0;
  int objective_evals = 0;
  bool converged = true;
  Scalar grad_norm = 0.0;
  Scalar trust_region_radius = 0.0;
  int scene_graph_rebuilds = 0;
  int scene_graph_reuses = 0;
  posegen_force_solver_stats_t force_solver;
};

struct posegen_result_t {
  pose_t optimal_pose;
  Matrix3X candidate_contact_forces;
  Scalar c_feq = 0.0;
  Scalar c_comp = 0.0;
  Scalar c_gap = 0.0;
  std::map<EntityId, Vector6> net_wrench;
  std::map<EntityId, std::vector<Vector3>> contact_force;
  std::map<EntityId, std::vector<Vector3>> contact_point;
  std::map<EntityId, std::vector<Vector3>> contact_normal;
  posegen_solver_stats_t solver;
};

}  // namespace stacking_core
