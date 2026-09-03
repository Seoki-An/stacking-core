#pragma once

#include <stacking_core/contact.hpp>
#include <stacking_core/scene/snapshot.hpp>

#include <map>
#include <memory>
#include <vector>

namespace stacking_core {

enum class simulation_contact_model_e {
  single_point,
  limit_surface_4d,
};

struct simulation_solver_config_t {
  // Reference scale used to normalise constraint rows against the body mass.
  // It does not regularise the velocity solve: the mass matrix does that, so
  // this value no longer damps motion.
  Scalar damping = 1e-3;
  Scalar beta_init = 1.0;
  int beta_update_interval = 30;
  Scalar beta_min = 1e-4;
  Scalar beta_max = 1e4;
  int max_iters = 2000;
  Scalar tol_abs = 1e-4;
  Scalar tol_rel = 1e-5;
  int stagnation_window = 50;
  Scalar stagnation_tol = 0.01;
  Scalar stagnation_min_metric = 0.0;
  Scalar stagnation_headroom = 1.5;
  Scalar stagnation_max_metric = 10.0;
};

struct simulation_contact_config_t {
  simulation_contact_model_e model =
    simulation_contact_model_e::single_point;
  Scalar error_reduction_ratio = 0.2;
  // Per-body override of error_reduction_ratio. A contact uses the larger of
  // its two bodies' ratios, so a group of bodies only loses penetration
  // correction among themselves; contacts against unlisted bodies keep the
  // default. Bodies absent from the map use error_reduction_ratio.
  std::map<EntityId, Scalar> body_error_reduction_ratio;
  Scalar detection_margin = 0.1;
  Scalar patch_eps = 1e-3;
  Scalar patch_damping = 1e-10;
  limit_surface_projection_config_t projection;
};

struct simulation_config_t {
  Vector3 gravity = Vector3 {0.0, 0.0, -9.81};
  simulation_contact_config_t contact;
  simulation_solver_config_t solver;
};

struct simulation_solver_stats_t {
  int iters = 0;
  bool converged = true;
  Scalar primal_residual = 0.0;
  Scalar dual_residual = 0.0;
};

struct simulation_result_t {
  std::shared_ptr<SceneSnapshot const> snapshot;
  std::vector<contact_t> contacts;
  simulation_solver_stats_t solver;
};

}  // namespace stacking_core
