#include <stacking_core/simulation.hpp>

#include <cmath>
#include <memory>
#include <source_location>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using stacking_core::Scalar;
using stacking_core::Vector3;

void require(
  bool condition,
  std::source_location location = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error(
      "simulation diffsim-reference requirement failed at line " +
      std::to_string(location.line()));
  }
}

bool near(Scalar first, Scalar second, Scalar tol = 2e-12) {
  return std::abs(first - second) <= tol;
}

stacking_core::Matrix3X octahedron(Scalar radius) {
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

stacking_core::Matrix3X cube_nodes() {
  using namespace stacking_core;
  Matrix3X nodes(3, 8);
  nodes <<
    -0.5, -0.5, -0.5, -0.5, 0.5, 0.5, 0.5, 0.5,
    -0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, 0.5,
    -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5;
  return nodes;
}

std::shared_ptr<stacking_core::BodyModel const> dynamic_model() {
  using namespace stacking_core;
  std::vector<geometry_config_t> geometries;
  geometries.push_back(dsf_vert_geometry_config_t {
    .properties = geometry_properties_t {
      .id = GeometryId {1},
      .body_from_geometry = pose_t {},
      .material = material_t {.friction = 0.6},
    },
    .nodes = octahedron(0.5),
    .sharpness = 20,
  });
  return std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {1},
    .inertial = inertial_t {
      .body_from_inertial = pose_t {},
      .mass = 2.0,
      .inertia = Vector3 {0.4, 0.6, 0.8}.asDiagonal(),
    },
    .geometries = std::move(geometries),
  });
}

std::shared_ptr<stacking_core::BodyModel const> plane_model() {
  using namespace stacking_core;
  std::vector<geometry_config_t> geometries;
  geometries.push_back(plane_geometry_config_t {
    .properties = geometry_properties_t {
      .id = GeometryId {2},
      .body_from_geometry = pose_t {},
      .material = material_t {.friction = 0.6},
    },
  });
  return std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {2},
    .inertial = std::nullopt,
    .geometries = std::move(geometries),
  });
}

std::shared_ptr<stacking_core::BodyModel const> moving_plane_model() {
  using namespace stacking_core;
  std::vector<geometry_config_t> geometries;
  geometries.push_back(plane_geometry_config_t {
    .properties = geometry_properties_t {
      .id = GeometryId {3},
      .body_from_geometry = pose_t {
        Vector3 {0.1, -0.2, 0.0}, Quaternion::Identity()},
      .material = material_t {.friction = 0.8},
    },
  });
  return std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {3},
    .inertial = std::nullopt,
    .geometries = std::move(geometries),
  });
}

std::shared_ptr<stacking_core::BodyModel const> moving_contact_model() {
  using namespace stacking_core;
  std::vector<geometry_config_t> geometries;
  geometries.push_back(dsf_vert_geometry_config_t {
    .properties = geometry_properties_t {
      .id = GeometryId {4},
      .body_from_geometry = pose_t {},
      .material = material_t {.friction = 0.6},
    },
    .nodes = cube_nodes(),
    .sharpness = 100,
  });
  return std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {4},
    .inertial = inertial_t {
      .body_from_inertial = pose_t {},
      .mass = 1.0,
      .inertia = Matrix3::Identity(),
    },
    .geometries = std::move(geometries),
  });
}

stacking_core::BodyInstance dynamic_body(Scalar z, Scalar vz) {
  using namespace stacking_core;
  return BodyInstance {body_instance_config_t {
    .id = EntityId {1},
    .model = dynamic_model(),
    .frame_from_body = pose_t {
      Vector3 {0.2, -0.1, z}, Quaternion::Identity()},
    .motion = motion_t {
      .linear = Vector3 {0.3, -0.2, vz},
      .angular = Vector3 {0.1, -0.3, 0.2},
    },
    .mobility = mobility_e::dynamic,
  }};
}

stacking_core::BodyInstance plane_body() {
  using namespace stacking_core;
  return BodyInstance {body_instance_config_t {
    .id = EntityId {2},
    .model = plane_model(),
    .frame_from_body = pose_t {},
    .motion = motion_t {},
    .mobility = mobility_e::static_body,
  }};
}

std::shared_ptr<stacking_core::SceneSnapshot const> free_scene() {
  using namespace stacking_core;
  std::vector<BodyInstance> bodies;
  bodies.push_back(dynamic_body(2.0, 0.4));
  return std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
    .frame = FrameId {1},
    .bodies = std::move(bodies),
  });
}

std::shared_ptr<stacking_core::SceneSnapshot const> contact_scene() {
  using namespace stacking_core;
  std::vector<BodyInstance> bodies;
  bodies.push_back(plane_body());
  bodies.push_back(dynamic_body(0.49, -0.2));
  return std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
    .frame = FrameId {1},
    .bodies = std::move(bodies),
  });
}

stacking_core::BodyInstance pair_body(
  stacking_core::EntityId id,
  Scalar x,
  Scalar vx,
  Scalar vy) {
  using namespace stacking_core;
  return BodyInstance {body_instance_config_t {
    .id = id,
    .model = dynamic_model(),
    .frame_from_body = pose_t {
      Vector3 {x, 0.0, 0.0}, Quaternion::Identity()},
    .motion = motion_t {
      .linear = Vector3 {vx, vy, 0.0},
      .angular = Vector3 {0.0, 0.0, 0.1},
    },
    .mobility = mobility_e::dynamic,
  }};
}

std::shared_ptr<stacking_core::SceneSnapshot const> pair_scene() {
  using namespace stacking_core;
  std::vector<BodyInstance> bodies;
  bodies.push_back(pair_body(EntityId {3}, -0.49, 0.2, 0.15));
  bodies.push_back(pair_body(EntityId {4}, 0.49, -0.1, -0.05));
  return std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
    .frame = FrameId {1},
    .bodies = std::move(bodies),
  });
}

std::shared_ptr<stacking_core::SceneSnapshot const> moving_contact_scene() {
  using namespace stacking_core;
  std::vector<BodyInstance> bodies;
  bodies.emplace_back(body_instance_config_t {
    .id = EntityId {5},
    .model = moving_plane_model(),
    .frame_from_body = pose_t {},
    .motion = motion_t {
      .linear = Vector3 {0.3, -0.2, 0.1},
      .angular = Vector3 {0.4, -0.3, 0.7},
    },
    .mobility = mobility_e::kinematic,
  });
  bodies.emplace_back(body_instance_config_t {
    .id = EntityId {6},
    .model = moving_contact_model(),
    .frame_from_body = pose_t {
      Vector3 {0.25, -0.1, 0.49}, Quaternion::Identity()},
    .motion = motion_t {},
    .mobility = mobility_e::dynamic,
  });
  return std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
    .frame = FrameId {1},
    .bodies = std::move(bodies),
  });
}

void require_state(
  stacking_core::BodyInstance const& body,
  Vector3 const& position,
  Eigen::Vector4d const& orientation_xyzw,
  Vector3 const& linear,
  Vector3 const& angular,
  Scalar tol = 2e-12) {
  Scalar const position_error =
    (body.frameFromBody().position - position).norm();
  Scalar const orientation_error =
    (body.frameFromBody().orientation.coeffs() - orientation_xyzw).norm();
  Scalar const linear_error = (body.motion().linear - linear).norm();
  Scalar const angular_error = (body.motion().angular - angular).norm();
  if (position_error > tol || orientation_error > tol ||
      linear_error > tol || angular_error > tol) {
    throw std::runtime_error(
      "simulation state error/tol ratios: position=" +
      std::to_string(position_error / tol) +
      ", orientation=" + std::to_string(orientation_error / tol) +
      ", linear=" + std::to_string(linear_error / tol) +
      ", angular=" + std::to_string(angular_error / tol));
  }
}

}  // namespace

int main() {
  using namespace stacking_core;

  // Regression reference, not a diffsim oracle. These values were emitted by
  // stacking-core itself and pin the current integrator and constraint solve
  // against unintended change.
  //
  // They previously came from diffsim and deliberately reproduced its
  // mass-normalized damping. That damping regularized the velocity solve and
  // acted as a compounding artificial drag -- at its default a free body kept
  // only 82% of its speed and 14% of its spin per simulated second. The
  // velocity update now applies the mass matrix directly instead of carrying
  // the equations of motion as a constraint factor, so free motion is exact
  // and diffsim's numbers are no longer reproducible.
  Simulator free_sim;
  simulation_result_t free_result = free_sim.step(*free_scene(), 0.01);
  require(free_result.contacts.empty());
  require_state(
    free_result.snapshot->body(EntityId {1}),
    Vector3 {
      0.20300000000000001,
      -0.10200000000000001,
      2.0030190000000001},
    Eigen::Vector4d {
      0.00049999970833338456,
      -0.0014999991250001531,
      0.00099999941666676913,
      0.99999825000051046},
    Vector3 {
      0.29999999999999993,
      -0.19999999999999998,
      0.30189999999999995},
    Vector3 {
      0.10000000000000002,
      -0.29999999999999999,
      0.20000000000000004});

  for (int iter = 1; iter < 10; ++iter) {
    free_result = free_sim.step(*free_result.snapshot, 0.01);
  }
  require_state(
    free_result.snapshot->body(EntityId {1}),
    Vector3 {
      0.23000000000000004,
      -0.12000000000000002,
      1.9860450000000001},
    Eigen::Vector4d {
      0.004999708338437458,
      -0.014999125015312368,
      0.0099994166768749178,
      0.9998250051041071},
    Vector3 {
      0.29999999999999977,
      -0.19999999999999996,
      -0.58099999999999985},
    Vector3 {
      0.10000000000000002,
      -0.29999999999999999,
      0.20000000000000004});

  Simulator free_batch_sim;
  simulation_result_t const free_batch =
    free_batch_sim.step_n(*free_scene(), 0.01, 10);
  require_state(
    free_batch.snapshot->body(EntityId {1}),
    Vector3 {
      0.23000000000000004,
      -0.12000000000000002,
      1.9860450000000001},
    Eigen::Vector4d {
      0.004999708338437458,
      -0.014999125015312368,
      0.0099994166768749178,
      0.9998250051041071},
    Vector3 {
      0.29999999999999977,
      -0.19999999999999996,
      -0.58099999999999985},
    Vector3 {
      0.10000000000000002,
      -0.29999999999999999,
      0.20000000000000004});

  Simulator contact_sim;
  simulation_result_t const contact_result =
    contact_sim.step(*contact_scene(), 0.01);
  require(contact_result.contacts.size() == 1);
  contact_t const& contact = contact_result.contacts.front();
  require(near(contact.feature.gap, -0.010000000000000009));
  require(contact.feature.normal.isApprox(Vector3::UnitZ(), 1e-12));
  require(contact.feature.point_first.isApprox(
    Vector3 {0.2, -0.1, 0.0}, 1e-12));
  require(contact.feature.point_second.isApprox(
    Vector3 {0.2, -0.1, -0.01}, 1e-12));
  require_state(
    contact_result.snapshot->body(EntityId {1}),
    Vector3 {
      0.2005454545434486,
      -0.10133333333324336,
      0.49199171948249726},
    Eigen::Vector4d {
      0.0013333326500462594,
      0.00054545426755362109,
      0.00099999948745033889,
      0.99999846235117351},
    Vector3 {
      0.054545454344860231,
      -0.13333333332433533,
      0.19917194824972795},
    Vector3 {
      0.26666666668916161,
      0.10909090942523292,
      0.20000000000000004},
    2e-12);

  simulation_config_t contact_4d_config;
  contact_4d_config.contact.model =
    simulation_contact_model_e::limit_surface_4d;
  Simulator contact_4d_sim {contact_4d_config};
  simulation_result_t const contact_4d_result =
    contact_4d_sim.step(*contact_scene(), 0.01);
  require(contact_4d_result.contacts.size() == 1);
  require_state(
    contact_4d_result.snapshot->body(EntityId {1}),
    Vector3 {
      0.20054545454541631,
      -0.10133333333333212,
      0.49199912578616667},
    Eigen::Vector4d {
      0.0013333326500860015,
      0.0005454542659755259,
      0.00099966035298289756,
      0.99999846269025172},
    Vector3 {
      0.05454545454162988,
      -0.13333333333321215,
      0.19991257861667017},
    Vector3 {
      0.26666666666696964,
      0.10909090909728354,
      0.19993217304914951},
    2e-12);

  simulation_config_t pair_config;
  pair_config.gravity.setZero();
  Simulator pair_sim {pair_config};
  simulation_result_t const pair_result = pair_sim.step(*pair_scene(), 0.01);
  require(pair_result.contacts.size() == 1);
  require(near(pair_result.contacts.front().feature.gap, -0.02));
  require(pair_result.contacts.front().feature.normal.isApprox(
    Vector3::UnitX(), 1e-12));
  require_state(
    pair_result.snapshot->body(EntityId {3}),
    Vector3 {
      -0.49149846643294243,
      0.00057677360188092676,
      0},
    Eigen::Vector4d {
      0,
      0,
      -7.7016498748282958e-05,
      0.99999999703422948},
    Vector3 {
      -0.14984664329424327,
      0.05767736018809267,
      0},
    Vector3 {
      0,
      0,
      -0.015403299764884143},
    2e-12);
  require_state(
    pair_result.snapshot->body(EntityId {4}),
    Vector3 {
      0.49249816737068336,
      0.00042292744804071892,
      0},
    Eigen::Vector4d {
      0,
      0,
      -7.6829654949864273e-05,
      0.9999999970486021},
    Vector3 {
      0.24981673706833751,
      0.042292744804071893,
      0},
    Vector3 {
      0,
      0,
      -0.015365931005089846},
    2e-12);

  simulation_config_t moving_config;
  moving_config.gravity.setZero();
  Simulator moving_sim {moving_config};
  simulation_result_t const moving_result =
    moving_sim.step(*moving_contact_scene(), 0.01);
  require(moving_result.contacts.size() == 1);
  require(near(
    moving_result.contacts.front().feature.gap,
    -0.016979739895014556));
  require_state(
    moving_result.snapshot->body(EntityId {5}),
    Vector3 {
      0.0030000000000000001,
      -0.002,
      0.001},
    Eigen::Vector4d {
      0.0019999938333390374,
      -0.0014999953750042781,
      0.003499989208343315,
      0.99999075001426041},
    Vector3 {
      0.29999999999999999,
      -0.20000000000000001,
      0.10000000000000001},
    Vector3 {
      0.40000000000000002,
      -0.29999999999999999,
      0.69999999999999996});
  require_state(
    moving_result.snapshot->body(EntityId {6}),
    Vector3 {
      0.25294327612021322,
      -0.10019887000812251,
      0.49474570000250734},
    Eigen::Vector4d {
      -5.0411527797142724e-05,
      -0.00074609061139771385,
      0,
      0.99999972040369967},
    Vector3 {
      0.29432761202131968,
      -0.019887000812251296,
      0.47457000025073676},
    Vector3 {
      -0.010082306499087096,
      -0.1492181361864893,
      0});

  moving_config.contact.model =
    simulation_contact_model_e::limit_surface_4d;
  Simulator moving_4d_sim {moving_config};
  simulation_result_t const moving_4d_result =
    moving_4d_sim.step(*moving_contact_scene(), 0.01);
  require(moving_4d_result.contacts.size() == 1);
  require_state(
    moving_4d_result.snapshot->body(EntityId {5}),
    Vector3 {
      0.0030000000000000001,
      -0.002,
      0.001},
    Eigen::Vector4d {
      0.0019999938333390374,
      -0.0014999953750042781,
      0.003499989208343315,
      0.99999075001426041},
    Vector3 {
      0.29999999999999999,
      -0.20000000000000001,
      0.10000000000000001},
    Vector3 {
      0.40000000000000002,
      -0.29999999999999999,
      0.69999999999999996});
  require_state(
    moving_4d_result.snapshot->body(EntityId {6}),
    Vector3 {
      0.25142366142040434,
      -0.10009619333921652,
      0.49474523139610666},
    Eigen::Vector4d {
      -2.4384029546753352e-05,
      -0.00036088363729194984,
      0.0013095407845204641,
      0.99999907713525082},
    Vector3 {
      0.14236614204043674,
      -0.0096193339216511294,
      0.47452313961066556},
    Vector3 {
      -0.0048768074095619776,
      -0.072176749661517298,
      0.26190823747272435});
}
