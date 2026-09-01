#include <stacking_core/kinematics/state.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace stacking_core {
namespace {

void validate_model(
  FrameId frame, std::shared_ptr<KinematicModel const> const& model) {
  if (!frame.valid()) {
    throw std::invalid_argument("kinematic state frame ID must be valid");
  }
  if (model == nullptr) {
    throw std::invalid_argument("kinematic state model must not be null");
  }
}

}  // namespace

KinematicState::KinematicState(
  FrameId frame, std::shared_ptr<KinematicModel const> model)
    : frame_(frame), model_(std::move(model)) {
  validate_model(frame_, model_);
  positions_ = Eigen::VectorXd::Zero(
    static_cast<Eigen::Index>(model_->degreeOfFreedomCount()));
  frame_from_roots_.resize(model_->roots().size());
}

KinematicState::KinematicState(kinematic_state_config_t config)
    : frame_(config.frame),
      model_(std::move(config.model)),
      positions_(std::move(config.positions)),
      frame_from_roots_(std::move(config.frame_from_roots)) {
  validate_model(frame_, model_);
  if (positions_.size() !=
      static_cast<Eigen::Index>(model_->degreeOfFreedomCount())) {
    throw std::invalid_argument("kinematic state position vector has the wrong size");
  }
  if (!positions_.allFinite()) {
    throw std::invalid_argument("kinematic state positions must be finite");
  }
  if (frame_from_roots_.size() != model_->roots().size()) {
    throw std::invalid_argument("kinematic state root pose count is incorrect");
  }
  for (pose_t const& pose : frame_from_roots_) {
    if (!is_valid(pose)) {
      throw std::invalid_argument("kinematic state root poses must be valid");
    }
  }
}

Scalar KinematicState::position(JointId joint) const {
  return model_->jointPosition(joint, positions_);
}

pose_t const& KinematicState::frameFromRoot(LinkId root) const {
  return frame_from_roots_.at(rootIndex(root));
}

bool KinematicState::positionsWithinLimits(Scalar tol) const {
  return model_->positionsWithinLimits(positions_, tol);
}

void KinematicState::setPositions(Eigen::Ref<Eigen::VectorXd const> positions) {
  if (positions.size() != positions_.size()) {
    throw std::invalid_argument("kinematic state position vector has the wrong size");
  }
  if (!positions.allFinite()) {
    throw std::invalid_argument("kinematic state positions must be finite");
  }
  positions_ = positions;
}

void KinematicState::setPosition(JointId joint, Scalar position_value) {
  if (!std::isfinite(position_value)) {
    throw std::invalid_argument("kinematic joint position must be finite");
  }
  kinematic_joint_t const& joint_model = model_->joint(joint);
  std::optional<std::size_t> const index = model_->degreeOfFreedomIndex(joint);
  if (!index.has_value() || joint_model.mimic.has_value()) {
    throw std::invalid_argument(
      "only an independent movable joint position can be set");
  }
  positions_[static_cast<Eigen::Index>(*index)] = position_value;
}

void KinematicState::setFrameFromRoot(
  LinkId root, pose_t frame_from_root) {
  if (!is_valid(frame_from_root)) {
    throw std::invalid_argument("kinematic root pose must be valid");
  }
  frame_from_roots_[rootIndex(root)] = std::move(frame_from_root);
}

std::size_t KinematicState::rootIndex(LinkId root) const {
  std::span<LinkId const> const roots = model_->roots();
  for (std::size_t index = 0; index < roots.size(); ++index) {
    if (roots[index] == root) {
      return index;
    }
  }
  throw std::out_of_range("kinematic link is not a model root");
}

}  // namespace stacking_core
