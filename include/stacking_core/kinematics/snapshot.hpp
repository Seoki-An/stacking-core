#pragma once

#include <stacking_core/kinematics/state.hpp>

#include <memory>
#include <vector>

namespace stacking_core {

// Immutable forward-kinematics result for one KinematicState.
class KinematicSnapshot {
public:
  [[nodiscard]] FrameId frame() const noexcept {
    return frame_;
  }

  [[nodiscard]] KinematicModel const& model() const noexcept {
    return *model_;
  }

  [[nodiscard]] std::shared_ptr<KinematicModel const> const& modelPtr() const noexcept {
    return model_;
  }

  [[nodiscard]] pose_t const& frameFromLink(std::size_t index) const;
  [[nodiscard]] pose_t const& frameFromLink(LinkId link) const;

  // Geometric Jacobian in the snapshot frame. Rows are [linear; angular]
  // velocity of the link origin and columns follow the model DoF ordering.
  [[nodiscard]] Matrix6X const& linkJacobian(std::size_t index) const;
  [[nodiscard]] Matrix6X const& linkJacobian(LinkId link) const;

private:
  friend KinematicSnapshot forward_kinematics(KinematicState const& state);

  KinematicSnapshot(
    FrameId frame,
    std::shared_ptr<KinematicModel const> model,
    std::vector<pose_t> frame_from_links,
    std::vector<Matrix6X> link_jacobians);

  FrameId frame_;
  std::shared_ptr<KinematicModel const> model_;
  std::vector<pose_t> frame_from_links_;
  std::vector<Matrix6X> link_jacobians_;
};

[[nodiscard]] KinematicSnapshot forward_kinematics(KinematicState const& state);

}  // namespace stacking_core
