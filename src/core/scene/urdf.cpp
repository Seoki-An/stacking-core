#include <stacking_core/scene/urdf.hpp>

#include <stacking_core/kinematics/snapshot.hpp>

#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace stacking_core {

SceneSnapshot make_urdf_scene_snapshot(
  UrdfModel const& model,
  KinematicState const& state,
  std::span<urdf_scene_binding_t const> bindings) {
  if (state.modelPtr() != model.kinematicsPtr()) {
    throw std::invalid_argument(
      "kinematic state must reference the URDF kinematic model");
  }
  if (bindings.size() != model.links().size()) {
    throw std::invalid_argument(
      "URDF scene bindings must contain every model link");
  }

  std::unordered_set<LinkId> link_ids;
  std::unordered_set<EntityId> entity_ids;
  for (urdf_scene_binding_t const& binding : bindings) {
    if (model.kinematics().findLink(binding.link) == nullptr) {
      throw std::invalid_argument(
        "URDF scene binding link must belong to the model");
    }
    if (!binding.entity.valid()) {
      throw std::invalid_argument("URDF scene entity ID must be valid");
    }
    if (!link_ids.emplace(binding.link).second) {
      throw std::invalid_argument("URDF scene link bindings must be unique");
    }
    if (!entity_ids.emplace(binding.entity).second) {
      throw std::invalid_argument("URDF scene entity IDs must be unique");
    }
  }

  KinematicSnapshot const poses = forward_kinematics(state);
  std::vector<BodyInstance> bodies;
  bodies.reserve(bindings.size());
  for (urdf_scene_binding_t const& binding : bindings) {
    bodies.emplace_back(body_instance_config_t {
      .id = binding.entity,
      .model = model.bodyModelPtr(binding.link),
      .frame_from_body = poses.frameFromLink(binding.link),
      .motion = motion_t {},
      .mobility = mobility_e::kinematic,
    });
  }

  return SceneSnapshot {scene_snapshot_config_t {
    .frame = state.frame(),
    .bodies = std::move(bodies),
  }};
}

}  // namespace stacking_core
