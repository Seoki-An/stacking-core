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

  // These values are emitted by the current diffsim Context using its default
  // solver configuration. They intentionally include its mass-normalized
  // damping, ADMM solve, and right-multiplied angular integration.
  Simulator free_sim;
  simulation_result_t free_result = free_sim.step(*free_scene(), 0.01);
  require(free_result.contacts.empty());
  require_state(
    free_result.snapshot->body(EntityId {1}),
    Vector3 {
      0.20299700299700302,
      -0.10199800199800201,
      2.0030159840159838},
    Eigen::Vector4d {
      0.00049751214945179315,
      -0.0014950157447812182,
      0.00099750565625746058,
      0.99999826119451318},
    Vector3 {
      0.29970029970029971,
      -0.19980019980019981,
      0.30159840159840162},
    Vector3 {
      0.099502487562189088,
      -0.29900332225913617,
      0.19950124688279305});

  for (int iter = 1; iter < 10; ++iter) {
    free_result = free_sim.step(*free_result.snapshot, 0.01);
  }
  require_state(
    free_result.snapshot->body(EntityId {1}),
    Vector3 {
      0.22983565786099108,
      -0.11989043857399402,
      1.9860409976920506},
    Eigen::Vector4d {
      0.0048647327780272642,
      -0.014727995062651518,
      0.0098631121884650147,
      0.99983105550607443},
    Vector3 {
      0.29701643421390073,
      -0.19801095614260056,
      -0.57960409976920513},
    Vector3 {
      0.095134794069607106,
      -0.29018091512525346,
      0.19506806804707916});

  Simulator free_batch_sim;
  simulation_result_t const free_batch =
    free_batch_sim.step_n(*free_scene(), 0.01, 10);
  require_state(
    free_batch.snapshot->body(EntityId {1}),
    Vector3 {
      0.22983565786099108,
      -0.11989043857399402,
      1.9860409976920506},
    Eigen::Vector4d {
      0.0048647327780272642,
      -0.014727995062651518,
      0.0098631121884650147,
      0.99983105550607443},
    Vector3 {
      0.29701643421390073,
      -0.19801095614260056,
      -0.57960409976920513},
    Vector3 {
      0.095134794069607106,
      -0.29018091512525346,
      0.19506806804707916});

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
      0.20054422815149697,
      -0.10132964187130826,
      0.49199797696312403},
    Eigen::Vector4d {
      0.0013296095776762704,
      0.00054418935823012172,
      0.00099750574104724753,
      0.99999847048813528},
    Vector3 {
      0.054422815149696491,
      -0.13296418713082481,
      0.19979769631240105},
    Vector3 {
      0.26592205111224532,
      0.10883792713566363,
      0.19950124992263618},
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
      0.20054422631860763,
      -0.10132964078801428,
      0.49199793947863496},
    Eigen::Vector4d {
      0.0013296215799153065,
      0.00054419299986603739,
      0.00099750405039171074,
      0.99999847047188151},
    Vector3 {
      0.054422631860760809,
      -0.13296407880142694,
      0.19979394786349658},
    Vector3 {
      0.26592445156271716,
      0.10883865546380776,
      0.19950091179243723},
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
      -0.49149510868803714,
      0.00057632229208779968,
      0.0},
    Eigen::Vector4d {
      0.0, 0.0, -7.6731962655561162e-05, 0.99999999705610298},
    Vector3 {
      -0.14951086880371228,
      0.057632229208779966,
      0.0},
    Vector3 {0.0, 0.0, -0.015346392546171633},
    2e-12);
  require_state(
    pair_result.snapshot->body(EntityId {4}),
    Vector3 {
      0.49249410967543822,
      0.00042267866842934698,
      0.0},
    Eigen::Vector4d {
      0.0, 0.0, -7.6731952953684843e-05, 0.99999999705610376},
    Vector3 {
      0.24941096754382175,
      0.042267866842934695,
      0.0},
    Vector3 {0.0, 0.0, -0.015346390605796361},
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
    Vector3 {0.003, -0.002, 0.001},
    Eigen::Vector4d {
      0.0019999938333390374,
      -0.0014999953750042781,
      0.003499989208343315,
      0.99999075001426041},
    Vector3 {0.3, -0.2, 0.1},
    Vector3 {0.4, -0.3, 0.7});
  require_state(
    moving_result.snapshot->body(EntityId {6}),
    Vector3 {
      0.25294094585438581,
      -0.10019871255772876,
      0.49473698273849376},
    Eigen::Vector4d {
      -5.0371615728458538e-05,
      -0.00074549991278119465,
      0.0,
      0.99999972084625122},
    Vector3 {
      0.29409458543857908,
      -0.019871255772875063,
      0.47369827384937629},
    Vector3 {
      -0.01007432408312017,
      -0.14909999643018018,
      0.0});

  moving_config.contact.model =
    simulation_contact_model_e::limit_surface_4d;
  Simulator moving_4d_sim {moving_config};
  simulation_result_t const moving_4d_result =
    moving_4d_sim.step(*moving_contact_scene(), 0.01);
  require(moving_4d_result.contacts.size() == 1);
  require_state(
    moving_4d_result.snapshot->body(EntityId {5}),
    Vector3 {0.003, -0.002, 0.001},
    Eigen::Vector4d {
      0.0019999938333390374,
      -0.0014999953750042781,
      0.003499989208343315,
      0.99999075001426041},
    Vector3 {0.3, -0.2, 0.1},
    Vector3 {0.4, -0.3, 0.7});
  require_state(
    moving_4d_result.snapshot->body(EntityId {6}),
    Vector3 {
      0.25142143771767483,
      -0.10009604308903211,
      0.49473699269697408},
    Eigen::Vector4d {
      -2.4345942685024471e-05,
      -0.0003603199517383774,
      0.0013072233370280276,
      0.99999908037155449},
    Vector3 {
      0.1421437717674833,
      -0.0096043089032101358,
      0.47369926969740883},
    Vector3 {
      -0.0048691900296202045,
      -0.072064012438382063,
      0.26144474754961933});
}
