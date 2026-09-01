#pragma once

#include <stacking_core/kinematics.hpp>
#include <stacking_core/planner/types.hpp>

#include <functional>
#include <limits>
#include <optional>

namespace stacking_core {

  using inverse_kinematics_initializer_t =
    std::function<std::optional<Eigen::VectorXd>(
      KinematicState const&, LinkId, pose_t const&)>;

  struct inverse_kinematics_problem_t {
    KinematicState initial_state;
    LinkId link;
    pose_t frame_from_link;
    bool position_only = false;
  };

  enum class inverse_kinematics_initialization_e {
    swing,
    provided,
  };

  struct inverse_kinematics_config_t {
    int max_iters = 100;
    Scalar tol = 1e-3;
    inverse_kinematics_initialization_e initialization =
      inverse_kinematics_initialization_e::swing;
  };

  struct inverse_kinematics_result_t {
    solve_status_e status = solve_status_e::invalid_problem;
    Eigen::VectorXd positions;
    pose_t frame_from_link;
    Scalar pos_error = std::numeric_limits<Scalar>::infinity();

    // The legacy orientation residual is 1-cos(theta), rather than theta.
    Scalar rot_error = std::numeric_limits<Scalar>::infinity();
    planner_solver_stats_t solver;
    planner_failure_t failure;
  };

  [[nodiscard]] inverse_kinematics_result_t solve_inverse_kinematics(
    inverse_kinematics_problem_t const& problem,
    inverse_kinematics_config_t const& config = {});

}  // namespace stacking_core
