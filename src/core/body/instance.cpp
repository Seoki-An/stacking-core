#include <stacking_core/body/instance.hpp>

#include <stdexcept>
#include <utility>

namespace stacking_core {

BodyInstance::BodyInstance(body_instance_config_t config)
    : id_(config.id),
      model_(std::move(config.model)),
      frame_from_body_(std::move(config.frame_from_body)),
      motion_(std::move(config.motion)),
      mobility_(config.mobility) {
  if (!id_.valid()) {
    throw std::invalid_argument("body instance ID must be valid");
  }
  if (model_ == nullptr) {
    throw std::invalid_argument("body instance model must not be null");
  }
  if (!is_valid(frame_from_body_)) {
    throw std::invalid_argument("frame_from_body must be a valid pose");
  }
  if (!is_finite(motion_)) {
    throw std::invalid_argument("body motion must be finite");
  }
}

pose_t BodyInstance::frameFromGeometry(GeometryId id) const {
  return compose(frame_from_body_, model().geometry(id).bodyFromGeometry());
}

void BodyInstance::setFrameFromBody(pose_t frame_from_body) {
  if (!is_valid(frame_from_body)) {
    throw std::invalid_argument("frame_from_body must be a valid pose");
  }
  frame_from_body_ = std::move(frame_from_body);
}

void BodyInstance::setMotion(motion_t motion) {
  if (!is_finite(motion)) {
    throw std::invalid_argument("body motion must be finite");
  }
  motion_ = std::move(motion);
}

}  // namespace stacking_core
