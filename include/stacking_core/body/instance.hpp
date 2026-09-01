#pragma once

#include <stacking_core/body/model.hpp>

#include <memory>

namespace stacking_core {

enum class mobility_e {
  static_body,
  kinematic,
  dynamic,
};

struct body_instance_config_t {
  EntityId id;
  std::shared_ptr<BodyModel const> model;
  pose_t frame_from_body;
  motion_t motion;
  mobility_e mobility = mobility_e::dynamic;
};

class BodyInstance {
public:
  explicit BodyInstance(body_instance_config_t config);

  [[nodiscard]] EntityId id() const noexcept {
    return id_;
  }

  [[nodiscard]] BodyModel const& model() const noexcept {
    return *model_;
  }

  [[nodiscard]] std::shared_ptr<BodyModel const> const& modelPtr() const noexcept {
    return model_;
  }

  [[nodiscard]] pose_t const& frameFromBody() const noexcept {
    return frame_from_body_;
  }

  [[nodiscard]] motion_t const& motion() const noexcept {
    return motion_;
  }

  [[nodiscard]] mobility_e mobility() const noexcept {
    return mobility_;
  }

  [[nodiscard]] bool isMovable() const noexcept {
    return mobility_ != mobility_e::static_body;
  }

  [[nodiscard]] bool isDynamic() const noexcept {
    return mobility_ == mobility_e::dynamic;
  }

  [[nodiscard]] pose_t frameFromGeometry(GeometryId id) const;

  void setFrameFromBody(pose_t frame_from_body);
  void setMotion(motion_t motion);
  void setMobility(mobility_e mobility) noexcept {
    mobility_ = mobility;
  }

private:
  EntityId id_;
  std::shared_ptr<BodyModel const> model_;
  pose_t frame_from_body_;
  motion_t motion_;
  mobility_e mobility_;
};

}  // namespace stacking_core
