#include <stacking_core/scene/snapshot.hpp>

#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace stacking_core {

SceneSnapshot::SceneSnapshot(scene_snapshot_config_t config)
    : frame_(config.frame), bodies_(std::move(config.bodies)) {
  if (!frame_.valid()) {
    throw std::invalid_argument("scene frame ID must be valid");
  }

  std::unordered_set<EntityId> entity_ids;
  for (BodyInstance const& body : bodies_) {
    if (!entity_ids.emplace(body.id()).second) {
      throw std::invalid_argument("scene body entity IDs must be unique");
    }
  }
}

BodyInstance const& SceneSnapshot::body(std::size_t index) const {
  return bodies_.at(index);
}

BodyInstance const& SceneSnapshot::body(EntityId id) const {
  BodyInstance const* result = findBody(id);
  if (result == nullptr) {
    throw std::out_of_range("scene does not contain the entity ID");
  }
  return *result;
}

BodyInstance const* SceneSnapshot::findBody(EntityId id) const noexcept {
  for (BodyInstance const& body : bodies_) {
    if (body.id() == id) {
      return &body;
    }
  }
  return nullptr;
}

}  // namespace stacking_core
