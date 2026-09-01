#pragma once

#include <stacking_core/kinematics/model.hpp>

#include <memory>
#include <vector>

namespace stacking_core {

struct kinematic_state_config_t {
  FrameId frame;
  std::shared_ptr<KinematicModel const> model;
  Eigen::VectorXd positions;
  std::vector<pose_t> frame_from_roots;
};

// Mutable generalized coordinates and root poses for one model instance.
class KinematicState {
public:
  KinematicState(
    FrameId frame, std::shared_ptr<KinematicModel const> model);
  explicit KinematicState(kinematic_state_config_t config);

  [[nodiscard]] FrameId frame() const noexcept {
    return frame_;
  }

  [[nodiscard]] KinematicModel const& model() const noexcept {
    return *model_;
  }

  [[nodiscard]] std::shared_ptr<KinematicModel const> const& modelPtr() const noexcept {
    return model_;
  }

  [[nodiscard]] Eigen::VectorXd const& positions() const noexcept {
    return positions_;
  }

  [[nodiscard]] Scalar position(JointId joint) const;
  [[nodiscard]] pose_t const& frameFromRoot(LinkId root) const;
  [[nodiscard]] bool positionsWithinLimits(Scalar tol = 0.0) const;

  void setPositions(Eigen::Ref<Eigen::VectorXd const> positions);
  void setPosition(JointId joint, Scalar position);
  void setFrameFromRoot(LinkId root, pose_t frame_from_root);

private:
  [[nodiscard]] std::size_t rootIndex(LinkId root) const;

  FrameId frame_;
  std::shared_ptr<KinematicModel const> model_;
  Eigen::VectorXd positions_;
  std::vector<pose_t> frame_from_roots_;
};

}  // namespace stacking_core
