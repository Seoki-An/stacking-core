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

stacking_core::Matrix3X octahedron(stacking_core::Scalar radius) {
  using namespace stacking_core;
  Matrix3X nodes(3, 6);
  nodes.col(0) = radius * Vector3::UnitX();
  nodes.col(1) = -radius * Vector3::UnitX();
  nodes.col(2) = radius * Vector3::UnitY();
  nodes.col(3) = -radius * Vector3::UnitY();
  nodes.col(4) = radius * Vector3::UnitZ();
  nodes.col(5) = -radius * Vector3::UnitZ();
  return nodes;
}

std::shared_ptr<stacking_core::BodyModel const> penetrating_model() {
  using namespace stacking_core;
  std::vector<geometry_config_t> geometries;
  geometries.push_back(dsf_vert_geometry_config_t {
    .properties = geometry_properties_t {
      .id = GeometryId {4},
      .body_from_geometry = pose_t {},
      .material = material_t {.friction = 0.6},
    },
    .nodes = octahedron(0.5),
    .sharpness = 20,
  });
  return std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {4},
    .inertial = inertial_t {
      .body_from_inertial = pose_t {},
      .mass = 2.0,
      .inertia = Vector3 {0.4, 0.6, 0.8}.asDiagonal(),
    },
    .geometries = std::move(geometries),
  });
}

std::shared_ptr<stacking_core::BodyModel const> ground_model() {
  using namespace stacking_core;
  std::vector<geometry_config_t> geometries;
  geometries.push_back(plane_geometry_config_t {
    .properties = geometry_properties_t {
      .id = GeometryId {5},
      .body_from_geometry = pose_t {},
      .material = material_t {.friction = 0.6},
    },
  });
  return std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {5},
    .inertial = std::nullopt,
    .geometries = std::move(geometries),
  });
}

// A body penetrating the ground by 10 mm, at rest.
stacking_core::SceneSnapshot penetrating_scene() {
  using namespace stacking_core;
  std::vector<BodyInstance> bodies;
  bodies.emplace_back(body_instance_config_t {
    .id = EntityId {10},
    .model = ground_model(),
    .frame_from_body = pose_t {},
    .motion = motion_t {},
    .mobility = mobility_e::static_body,
  });
  bodies.emplace_back(body_instance_config_t {
    .id = EntityId {11},
    .model = penetrating_model(),
    .frame_from_body = pose_t {
      Vector3 {0.0, 0.0, 0.49}, Quaternion::Identity()},
    .motion = motion_t {},
    .mobility = mobility_e::dynamic,
  });
  return SceneSnapshot {scene_snapshot_config_t {
    .frame = FrameId {1},
    .bodies = std::move(bodies),
  }};
}

stacking_core::Scalar separation_speed(
  stacking_core::simulation_config_t config) {
  using namespace stacking_core;
  Simulator simulator {std::move(config)};
  SceneSnapshot const scene = penetrating_scene();
  simulation_result_t const result = simulator.step(scene, 0.01);
  return result.snapshot->body(EntityId {11}).motion().linear.z();
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

  // Per-body error_reduction_ratio: a contact uses the larger of its two
  // bodies' ratios, so overriding only one body leaves the contact unchanged.
  simulation_config_t erp_config;
  Scalar const default_speed = separation_speed(erp_config);
  require(default_speed > 0.0);

  simulation_config_t one_override = erp_config;
  one_override.contact.body_error_reduction_ratio.emplace(EntityId {11}, 0.0);
  require(separation_speed(one_override) == default_speed);

  simulation_config_t both_override = one_override;
  both_override.contact.body_error_reduction_ratio.emplace(EntityId {10}, 0.0);
  require(separation_speed(both_override) < default_speed);

  simulation_config_t raised = erp_config;
  raised.contact.body_error_reduction_ratio.emplace(EntityId {10}, 1.0);
  require(separation_speed(raised) > default_speed);

  bool rejected_body_ratio = false;
  try {
    simulation_config_t invalid = erp_config;
    invalid.contact.body_error_reduction_ratio.emplace(EntityId {10}, 1.5);
    (void)Simulator {invalid};
  } catch (std::invalid_argument const&) {
    rejected_body_ratio = true;
  }
  require(rejected_body_ratio);
}
