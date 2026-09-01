#include <stacking_core/scene/view.hpp>

#include <algorithm>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace stacking_core {

SceneView::SceneView(
  std::shared_ptr<SceneSnapshot const> snapshot,
  std::vector<EntityId> entity_ids)
    : snapshot_(std::move(snapshot)), entity_ids_(std::move(entity_ids)) {
  if (snapshot_ == nullptr) {
    throw std::invalid_argument("scene view snapshot must not be null");
  }

  std::unordered_set<EntityId> unique_ids;
  for (EntityId id : entity_ids_) {
    if (!unique_ids.emplace(id).second) {
      throw std::invalid_argument("scene view entity IDs must be unique");
    }
    if (snapshot_->findBody(id) == nullptr) {
      throw std::invalid_argument("scene view entity ID is not in the snapshot");
    }
  }
}

bool SceneView::contains(EntityId id) const noexcept {
  return std::find(entity_ids_.begin(), entity_ids_.end(), id) !=
    entity_ids_.end();
}

BodyInstance const& SceneView::body(std::size_t index) const {
  return snapshot_->body(entity_ids_.at(index));
}

BodyInstance const& SceneView::body(EntityId id) const {
  BodyInstance const* result = findBody(id);
  if (result == nullptr) {
    throw std::out_of_range("scene view does not contain the entity ID");
  }
  return *result;
}

BodyInstance const* SceneView::findBody(EntityId id) const noexcept {
  if (!contains(id)) {
    return nullptr;
  }
  return snapshot_->findBody(id);
}

}  // namespace stacking_core
