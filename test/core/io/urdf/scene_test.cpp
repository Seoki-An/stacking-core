#include <stacking_core/io/urdf.hpp>
#include <stacking_core/kinematics.hpp>
#include <stacking_core/scene/urdf.hpp>

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("URDF scene test requirement failed");
  }
}

std::filesystem::path asset(std::string const& name) {
  return std::filesystem::path {__FILE__}.parent_path() / "assets" / name;
}

stacking_core::EntityId entity_for(stacking_core::LinkId link) {
  return stacking_core::EntityId {100 + link.value()};
}

}  // namespace

int main() {
  using namespace stacking_core;

  UrdfModel model = load_urdf_model(asset("robot.urdf"));
  LinkId const world = model.kinematics().findLink("world")->id;
  LinkId const base = model.kinematics().findLink("base")->id;
  LinkId const arm = model.kinematics().findLink("arm")->id;
  JointId const shoulder = model.kinematics().findJoint("shoulder")->id;

  KinematicState state {FrameId {42}, model.kinematicsPtr()};
  state.setFrameFromRoot(world, pose_t {});
  state.setFrameFromRoot(
    base, pose_t {Vector3 {10.0, 0.0, 0.0}, Quaternion::Identity()});
  state.setPosition(shoulder, 0.5);

  std::vector<urdf_scene_binding_t> bindings;
  for (auto link = model.links().rbegin(); link != model.links().rend(); ++link) {
    bindings.push_back(urdf_scene_binding_t {
      .link = link->link,
      .entity = entity_for(link->link),
    });
  }

  SceneSnapshot const scene = make_urdf_scene_snapshot(model, state, bindings);
  KinematicSnapshot const poses = forward_kinematics(state);
  require(scene.frame() == FrameId {42});
  require(scene.bodyCount() == model.links().size());
  require(scene.body(0).id() == bindings[0].entity);
  require(scene.body(entity_for(arm)).frameFromBody().position.isApprox(
    poses.frameFromLink(arm).position));
  require(scene.body(entity_for(base)).modelPtr() == model.bodyModelPtr(base));
  require(scene.body(entity_for(base)).mobility() == mobility_e::kinematic);
  require(is_finite(scene.body(entity_for(base)).motion()));
  require(scene.body(entity_for(base)).motion().linear.isZero());

  bool rejected_missing = false;
  try {
    std::vector<urdf_scene_binding_t> incomplete = bindings;
    incomplete.pop_back();
    SceneSnapshot const invalid =
      make_urdf_scene_snapshot(model, state, incomplete);
    (void)invalid;
  } catch (std::invalid_argument const&) {
    rejected_missing = true;
  }
  require(rejected_missing);

  bool rejected_duplicate_entity = false;
  try {
    std::vector<urdf_scene_binding_t> duplicates = bindings;
    duplicates[1].entity = duplicates[0].entity;
    SceneSnapshot const invalid =
      make_urdf_scene_snapshot(model, state, duplicates);
    (void)invalid;
  } catch (std::invalid_argument const&) {
    rejected_duplicate_entity = true;
  }
  require(rejected_duplicate_entity);

  bool rejected_other_model = false;
  try {
    auto other_model = std::make_shared<KinematicModel>(kinematic_model_config_t {
      .links = {kinematic_link_t {.id = LinkId {90}, .name = "other"}},
      .joints = {},
    });
    KinematicState const other_state {FrameId {42}, std::move(other_model)};
    SceneSnapshot const invalid =
      make_urdf_scene_snapshot(model, other_state, bindings);
    (void)invalid;
  } catch (std::invalid_argument const&) {
    rejected_other_model = true;
  }
  require(rejected_other_model);
}
