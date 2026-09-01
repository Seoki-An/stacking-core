#pragma once

#include <stacking_core/scene/snapshot.hpp>

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace stacking_core {

class SceneView {
public:
  SceneView(
    std::shared_ptr<SceneSnapshot const> snapshot,
    std::vector<EntityId> entity_ids);

  [[nodiscard]] SceneSnapshot const& snapshot() const noexcept {
    return *snapshot_;
  }

  [[nodiscard]] std::shared_ptr<SceneSnapshot const> const& snapshotPtr()
    const noexcept {
    return snapshot_;
  }

  [[nodiscard]] FrameId frame() const noexcept {
    return snapshot_->frame();
  }

  [[nodiscard]] std::size_t bodyCount() const noexcept {
    return entity_ids_.size();
  }

  [[nodiscard]] std::span<EntityId const> entityIds() const noexcept {
    return entity_ids_;
  }

  [[nodiscard]] bool contains(EntityId id) const noexcept;
  [[nodiscard]] BodyInstance const& body(std::size_t index) const;
  [[nodiscard]] BodyInstance const& body(EntityId id) const;
  [[nodiscard]] BodyInstance const* findBody(EntityId id) const noexcept;

private:
  std::shared_ptr<SceneSnapshot const> snapshot_;
  std::vector<EntityId> entity_ids_;
};

}  // namespace stacking_core
