#pragma once

#include <stacking_core/body.hpp>

#include <cstddef>
#include <vector>

namespace stacking_core {

struct scene_snapshot_config_t {
  FrameId frame;
  std::vector<BodyInstance> bodies;
};

class SceneSnapshot {
public:
  explicit SceneSnapshot(scene_snapshot_config_t config);

  SceneSnapshot(SceneSnapshot const&) = delete;
  SceneSnapshot& operator=(SceneSnapshot const&) = delete;
  SceneSnapshot(SceneSnapshot&&) = delete;
  SceneSnapshot& operator=(SceneSnapshot&&) = delete;

  [[nodiscard]] FrameId frame() const noexcept {
    return frame_;
  }

  [[nodiscard]] std::size_t bodyCount() const noexcept {
    return bodies_.size();
  }

  [[nodiscard]] BodyInstance const& body(std::size_t index) const;
  [[nodiscard]] BodyInstance const& body(EntityId id) const;
  [[nodiscard]] BodyInstance const* findBody(EntityId id) const noexcept;

private:
  FrameId frame_;
  std::vector<BodyInstance> bodies_;
};

}  // namespace stacking_core
