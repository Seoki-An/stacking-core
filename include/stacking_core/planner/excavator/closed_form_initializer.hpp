#pragma once

#include <stacking_core/planner/inverse_kinematics.hpp>

#include <array>
#include <memory>

namespace stacking_core {

  // Explicitly identifies the VDK23_CX-style six-joint chain. This extension
  // never infers compatibility from joint ordering alone.
  struct excavator_ik_chain_t {
    JointId swing;
    JointId boom;
    JointId arm;
    JointId bucket;
    JointId tilt;
    JointId rotate;
    LinkId end_link;

    // The requested target is frame_from_task. The closed-form equations use
    // frame_from_end, obtained as frame_from_task * inverse(end_from_task).
    pose_t end_from_task;
  };

  struct excavator_ik_seed_result_t {
    solve_status_e status = solve_status_e::invalid_problem;
    Eigen::VectorXd positions;
    planner_failure_t failure;
  };

  // Optional, model-specific seed provider. It contains no LM refinement;
  // callers pass a successful seed to solve_inverse_kinematics().
  class ExcavatorIkInitializer {
  public:
    ExcavatorIkInitializer(
      std::shared_ptr<KinematicModel const> model, excavator_ik_chain_t chain);

    [[nodiscard]] excavator_ik_seed_result_t seed(
      KinematicState const& state, pose_t const& frame_from_task) const;

  private:
    std::shared_ptr<KinematicModel const> model_;
    excavator_ik_chain_t chain_;
    std::array<std::size_t, 6> dof_indices_;
  };

}  // namespace stacking_core
