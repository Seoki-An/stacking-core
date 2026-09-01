#pragma once

#include <stacking_core/collision/types.hpp>
#include <stacking_core/scene/view.hpp>

#include <Eigen/Core>

#include <optional>
#include <string>
#include <vector>

namespace stacking_core {

  enum class solve_status_e {
    success,
    infeasible,
    max_iters,
    cancelled,
    invalid_problem,
  };

  enum class motion_mode_e {
    free,
    attached,
  };

  enum class grasp_event_e {
    acquire,
    release,
  };

  struct planner_failure_t {
    std::string code;
    std::string message;
    bool retryable = false;
  };

  struct planner_solver_stats_t {
    int iters = 0;
    bool converged = false;
    Scalar objective = 0.0;
    Scalar grad_norm = 0.0;
  };

  // The gripper model maps this scalar opening coordinate to its independent
  // joints. It may represent an angle or a translation depending on the hand.
  struct grasp_t {
    pose_t frame_from_grasp;
    Scalar opening = 0.0;
  };

  struct robot_state_t {
    Eigen::VectorXd positions;
    Scalar gripper_opening = 0.0;
  };

  // A solve-specific scene role. Different phases may refer to different
  // immutable snapshots, but every target belongs to its phase's scene.
  struct phase_scene_t {
    SceneView scene;
    EntityId target;
  };

  struct attachment_t {
    EntityId body;
    LinkId link;
    pose_t link_from_body;
  };

  struct trajectory_sample_t {
    robot_state_t robot;
    std::optional<pose_t> frame_from_target;
  };

  struct trajectory_t {
    std::vector<trajectory_sample_t> samples;
  };

  struct grasp_contact_t {
    contact_feature_t feature;
    std::optional<Vector3> force;
  };

}  // namespace stacking_core
