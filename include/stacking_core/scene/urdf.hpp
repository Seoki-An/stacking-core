#pragma once

#include <stacking_core/io/urdf.hpp>
#include <stacking_core/kinematics/state.hpp>
#include <stacking_core/scene/snapshot.hpp>

#include <span>

namespace stacking_core {

struct urdf_scene_binding_t {
  LinkId link;
  EntityId entity;
};

// Instantiates every URDF link as a kinematic scene body. Entity IDs are
// supplied explicitly so multiple robot instances can safely share a scene.
[[nodiscard]] SceneSnapshot make_urdf_scene_snapshot(
  UrdfModel const& model,
  KinematicState const& state,
  std::span<urdf_scene_binding_t const> bindings);

}  // namespace stacking_core
