#include <stacking_core/scene/view.hpp>

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("scene view test requirement failed");
  }
}

std::shared_ptr<stacking_core::BodyModel const> make_model() {
  using namespace stacking_core;
  std::vector<geometry_config_t> geometries;
  geometries.push_back(point_geometry_config_t {
    .properties = {
      .id = GeometryId {1},
      .body_from_geometry = pose_t {},
      .material = material_t {},
    },
  });
  return std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {1},
    .inertial = inertial_t {},
    .geometries = std::move(geometries),
  });
}

stacking_core::BodyInstance make_body(
  stacking_core::EntityId id,
  std::shared_ptr<stacking_core::BodyModel const> model) {
  using namespace stacking_core;
  return BodyInstance {body_instance_config_t {
    .id = id,
    .model = std::move(model),
    .frame_from_body = pose_t {},
    .motion = motion_t {},
    .mobility = mobility_e::static_body,
  }};
}

std::shared_ptr<stacking_core::SceneSnapshot const> make_snapshot() {
  using namespace stacking_core;
  std::shared_ptr<BodyModel const> const model = make_model();
  std::vector<BodyInstance> bodies;
  bodies.push_back(make_body(EntityId {10}, model));
  bodies.push_back(make_body(EntityId {11}, model));
  bodies.push_back(make_body(EntityId {12}, model));
  return std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
    .frame = FrameId {3},
    .bodies = std::move(bodies),
  });
}

}  // namespace

int main() {
  using namespace stacking_core;

  std::shared_ptr<SceneSnapshot const> const snapshot = make_snapshot();
  SceneView view {snapshot, {EntityId {12}, EntityId {10}}};
  require(view.snapshotPtr() == snapshot);
  require(view.frame() == FrameId {3});
  require(view.bodyCount() == 2);
  require(view.entityIds()[0] == EntityId {12});
  require(view.entityIds()[1] == EntityId {10});
  require(view.body(0).id() == EntityId {12});
  require(view.body(EntityId {10}).id() == EntityId {10});
  require(view.contains(EntityId {12}));
  require(!view.contains(EntityId {11}));
  require(view.findBody(EntityId {11}) == nullptr);

  SceneView const empty {snapshot, {}};
  require(empty.bodyCount() == 0);

  bool rejected_duplicate = false;
  try {
    SceneView const invalid_view {
      snapshot, {EntityId {10}, EntityId {10}}};
    (void)invalid_view;
  } catch (std::invalid_argument const&) {
    rejected_duplicate = true;
  }
  require(rejected_duplicate);

  bool rejected_unknown = false;
  try {
    SceneView const invalid_view {snapshot, {EntityId {99}}};
    (void)invalid_view;
  } catch (std::invalid_argument const&) {
    rejected_unknown = true;
  }
  require(rejected_unknown);

  bool rejected_null = false;
  try {
    SceneView const invalid_view {nullptr, {}};
    (void)invalid_view;
  } catch (std::invalid_argument const&) {
    rejected_null = true;
  }
  require(rejected_null);
}
