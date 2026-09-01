#pragma once

#include <stacking_core/body/model.hpp>
#include <stacking_core/planner/types.hpp>

#include <memory>
#include <vector>

namespace stacking_core {

  struct stable_pose_problem_t {
    std::shared_ptr<BodyModel const> body_model;
    // Offset from the combined DSF-node centroid, expressed in the body frame.
    Vector3 com_offset_body = Vector3::Zero();
  };

  struct stable_pose_config_t {
    int sampling_level = 2;
    Scalar stable_eigenvalue_min = -0.2;
  };

  struct stable_pose_t {
    Vector3 resting_dir_body = Vector3::UnitZ();
    Scalar cone_angle = 0.0;
    Vector3 support_point_body = Vector3::Zero();
  };

  struct stable_pose_result_t {
    solve_status_e status = solve_status_e::invalid_problem;
    std::vector<stable_pose_t> poses;
    // Body-frame point about which resting directions and support heights are
    // measured. Regrasp placement uses it to put the support point on a table
    // without reconstructing the stable-pose envelope.
    Vector3 reference_point_body = Vector3::Zero();
    planner_solver_stats_t solver;
    planner_failure_t failure;
  };

  [[nodiscard]] stable_pose_result_t solve_stable_poses(
    stable_pose_problem_t const& problem,
    stable_pose_config_t const& config = {});

}  // namespace stacking_core
