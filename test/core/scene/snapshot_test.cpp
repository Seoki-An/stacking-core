#include <stacking_core/scene/snapshot.hpp>

#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

using stacking_core::Vector3;

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("scene snapshot test requirement failed");
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
  std::shared_ptr<stacking_core::BodyModel const> model,
  stacking_core::Vector3 position) {
  using namespace stacking_core;
  return BodyInstance {body_instance_config_t {
    .id = id,
    .model = std::move(model),
    .frame_from_body = pose_t {position, Quaternion::Identity()},
    .motion = motion_t {},
    .mobility = mobility_e::dynamic,
  }};
}

}  // namespace

int main() {
  using namespace stacking_core;

  static_assert(std::is_same_v<
                decltype(std::declval<SceneSnapshot const&>().body(EntityId {})),
                BodyInstance const&>);

  std::shared_ptr<BodyModel const> const model = make_model();
  std::vector<BodyInstance> bodies;
  bodies.push_back(make_body(EntityId {10}, model, Vector3 {1.0, 0.0, 0.0}));
  bodies.push_back(make_body(EntityId {11}, model, Vector3 {2.0, 0.0, 0.0}));
  SceneSnapshot snapshot {{
    .frame = FrameId {7},
    .bodies = std::move(bodies),
  }};

  require(snapshot.frame() == FrameId {7});
  require(snapshot.bodyCount() == 2);
  require(snapshot.body(0).id() == EntityId {10});
  require(snapshot.body(EntityId {11}).id() == EntityId {11});
  require(snapshot.findBody(EntityId {12}) == nullptr);
  require(snapshot.body(EntityId {10}).frameFromBody().position.isApprox(
    Vector3 {1.0, 0.0, 0.0}));

  SceneSnapshot const empty {{
    .frame = FrameId {8},
    .bodies = {},
  }};
  require(empty.bodyCount() == 0);

  bool rejected_frame = false;
  try {
    SceneSnapshot const invalid_snapshot {{
      .frame = FrameId {},
      .bodies = {},
    }};
    (void)invalid_snapshot;
  } catch (std::invalid_argument const&) {
    rejected_frame = true;
  }
  require(rejected_frame);

  bool rejected_duplicate = false;
  try {
    std::vector<BodyInstance> duplicates;
    duplicates.push_back(
      make_body(EntityId {20}, model, Vector3 {0.0, 0.0, 0.0}));
    duplicates.push_back(
      make_body(EntityId {20}, model, Vector3 {1.0, 0.0, 0.0}));
    SceneSnapshot const invalid_snapshot {{
      .frame = FrameId {9},
      .bodies = std::move(duplicates),
    }};
    (void)invalid_snapshot;
  } catch (std::invalid_argument const&) {
    rejected_duplicate = true;
  }
  require(rejected_duplicate);
}
