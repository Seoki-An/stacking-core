#include <stacking_core/simulation.hpp>

#include <memory>
#include <source_location>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

void require(
  bool condition,
  std::source_location location = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error(
      "simulation test requirement failed at line " +
      std::to_string(location.line()));
  }
}

std::shared_ptr<stacking_core::BodyModel const> model_without_inertial() {
  using namespace stacking_core;
  std::vector<geometry_config_t> geometries;
  geometries.push_back(point_geometry_config_t {
    .properties = geometry_properties_t {
      .id = GeometryId {1},
      .body_from_geometry = pose_t {},
      .material = material_t {},
    },
  });
  return std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {1},
    .inertial = std::nullopt,
    .geometries = std::move(geometries),
  });
}

}  // namespace

int main() {
  using namespace stacking_core;

  pose_t const integrated = integrate_pose(
    pose_t {Vector3 {1.0, 2.0, 3.0}, Quaternion::Identity()},
    motion_t {
      .linear = Vector3 {0.5, -1.0, 2.0},
      .angular = Vector3::UnitZ(),
    },
    0.2);
  require(integrated.position.isApprox(Vector3 {1.1, 1.8, 3.4}));
  require(integrated.orientation.isApprox(Quaternion {
    Eigen::AngleAxisd(0.2, Vector3::UnitZ())}));

  bool rejected_negative_dt = false;
  try {
    (void)integrate_pose(pose_t {}, motion_t {}, -0.1);
  } catch (std::invalid_argument const&) {
    rejected_negative_dt = true;
  }
  require(rejected_negative_dt);

  std::shared_ptr<BodyModel const> const model = model_without_inertial();
  std::vector<BodyInstance> bodies;
  bodies.emplace_back(body_instance_config_t {
    .id = EntityId {1},
    .model = model,
    .frame_from_body = pose_t {},
    .motion = motion_t {.linear = Vector3::UnitX()},
    .mobility = mobility_e::kinematic,
  });
  bodies.emplace_back(body_instance_config_t {
    .id = EntityId {2},
    .model = model,
    .frame_from_body = pose_t {Vector3::UnitY(), Quaternion::Identity()},
    .motion = motion_t {.linear = Vector3::UnitY()},
    .mobility = mobility_e::static_body,
  });
  SceneSnapshot const scene {scene_snapshot_config_t {
    .frame = FrameId {1},
    .bodies = std::move(bodies),
  }};
  Simulator simulator;
  simulation_result_t const result = simulator.step(scene, 0.1);
  require(result.snapshot->body(EntityId {1}).frameFromBody().position.isApprox(
    Vector3 {0.1, 0.0, 0.0}));
  require(result.snapshot->body(EntityId {1}).motion().linear.isApprox(
    Vector3::UnitX()));
  require(result.snapshot->body(EntityId {2}).frameFromBody().position.isApprox(
    Vector3::UnitY()));

  Simulator repeated_sim;
  simulation_result_t repeated = repeated_sim.step(scene, 0.1);
  repeated = repeated_sim.step(*repeated.snapshot, 0.1);
  repeated = repeated_sim.step(*repeated.snapshot, 0.1);
  Simulator batch_sim;
  simulation_result_t const batch = batch_sim.step_n(scene, 0.1, 3);
  BodyInstance const& repeated_body =
    repeated.snapshot->body(EntityId {1});
  BodyInstance const& batch_body = batch.snapshot->body(EntityId {1});
  require(batch_body.frameFromBody().position.isApprox(
    repeated_body.frameFromBody().position, 0.0));
  require(batch_body.frameFromBody().orientation.coeffs().isApprox(
    repeated_body.frameFromBody().orientation.coeffs(), 0.0));
  require(batch_body.motion().linear.isApprox(
    repeated_body.motion().linear, 0.0));
  require(batch_body.mobility() == mobility_e::kinematic);

  simulation_result_t const unchanged = simulator.step_n(scene, -1.0, 0);
  require(unchanged.snapshot.get() != &scene);
  require(unchanged.snapshot->frame() == scene.frame());
  require(unchanged.snapshot->bodyCount() == scene.bodyCount());
  require(unchanged.snapshot->body(EntityId {1}).modelPtr() ==
    scene.body(EntityId {1}).modelPtr());
  require(unchanged.snapshot->body(EntityId {1}).frameFromBody().position.isApprox(
    scene.body(EntityId {1}).frameFromBody().position, 0.0));
  require(unchanged.snapshot->body(EntityId {2}).motion().linear.isApprox(
    scene.body(EntityId {2}).motion().linear, 0.0));
  require(unchanged.contacts.empty());
  require(unchanged.solver.iters == 0);
  require(unchanged.solver.converged);

  bool rejected_batch_dt = false;
  try {
    (void)simulator.step_n(scene, -0.1, 1);
  } catch (std::invalid_argument const&) {
    rejected_batch_dt = true;
  }
  require(rejected_batch_dt);

  std::vector<BodyInstance> invalid_bodies;
  invalid_bodies.emplace_back(body_instance_config_t {
    .id = EntityId {3},
    .model = model,
    .frame_from_body = pose_t {},
    .motion = motion_t {},
    .mobility = mobility_e::dynamic,
  });
  SceneSnapshot const invalid_scene {scene_snapshot_config_t {
    .frame = FrameId {1},
    .bodies = std::move(invalid_bodies),
  }};
  bool rejected_missing_inertial = false;
  try {
    (void)simulator.step(invalid_scene, 0.1);
  } catch (std::invalid_argument const&) {
    rejected_missing_inertial = true;
  }
  require(rejected_missing_inertial);
}
