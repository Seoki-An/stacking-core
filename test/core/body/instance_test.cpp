#include <stacking_core/body/instance.hpp>
#include <stacking_core/geometry/point.hpp>

#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace {

using stacking_core::Vector3;

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("body instance test requirement failed");
  }
}

bool near(Vector3 const& lhs, Vector3 const& rhs) {
  return (lhs - rhs).norm() < 1e-12;
}

std::shared_ptr<stacking_core::BodyModel const> make_model() {
  using namespace stacking_core;
  std::vector<geometry_config_t> geometries;
  geometries.push_back(point_geometry_config_t {
    .properties = {
      .id = GeometryId {1},
      .body_from_geometry =
        pose_t {Vector3 {1.0, 0.0, 0.0}, Quaternion::Identity()},
      .material = material_t {},
    },
  });
  return std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {1},
    .inertial = inertial_t {},
    .geometries = std::move(geometries),
  });
}

}  // namespace

int main() {
  using namespace stacking_core;

  std::shared_ptr<BodyModel const> const model = make_model();
  BodyInstance first {{
    .id = EntityId {10},
    .model = model,
    .frame_from_body =
      pose_t {Vector3 {0.0, 2.0, 0.0}, Quaternion::Identity()},
    .motion = motion_t {},
    .mobility = mobility_e::dynamic,
  }};
  BodyInstance second {{
    .id = EntityId {11},
    .model = model,
    .frame_from_body =
      pose_t {Vector3 {0.0, 5.0, 0.0}, Quaternion::Identity()},
    .motion = motion_t {},
    .mobility = mobility_e::static_body,
  }};

  require(first.modelPtr() == second.modelPtr());
  require(first.isMovable());
  require(first.isDynamic());
  require(!second.isMovable());
  require(!second.isDynamic());
  require(near(
    first.frameFromGeometry(GeometryId {1}).position,
    Vector3 {1.0, 2.0, 0.0}));
  require(near(
    second.frameFromGeometry(GeometryId {1}).position,
    Vector3 {1.0, 5.0, 0.0}));

  first.setFrameFromBody(
    pose_t {Vector3 {3.0, 0.0, 0.0}, Quaternion::Identity()});
  require(near(first.frameFromBody().position, Vector3 {3.0, 0.0, 0.0}));
  first.setMobility(mobility_e::kinematic);
  require(first.isMovable());
  require(!first.isDynamic());

  motion_t moving;
  moving.linear = Vector3 {1.0, 2.0, 3.0};
  first.setMotion(moving);
  require(near(first.motion().linear, moving.linear));

  bool rejected_null_model = false;
  try {
    BodyInstance const invalid_instance {{
      .id = EntityId {12},
      .model = nullptr,
      .frame_from_body = pose_t {},
      .motion = motion_t {},
      .mobility = mobility_e::dynamic,
    }};
    (void)invalid_instance;
  } catch (std::invalid_argument const&) {
    rejected_null_model = true;
  }
  require(rejected_null_model);

  bool rejected_invalid_motion = false;
  try {
    motion_t invalid_motion;
    invalid_motion.angular.z() = std::numeric_limits<Scalar>::infinity();
    first.setMotion(invalid_motion);
  } catch (std::invalid_argument const&) {
    rejected_invalid_motion = true;
  }
  require(rejected_invalid_motion);
  require(is_finite(first.motion()));
}
