#include <stacking_core/simulation.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <source_location>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace stacking_core;

void require(
  bool condition,
  std::string const& what,
  std::source_location location = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error(
      "simulation stability requirement failed at line " +
      std::to_string(location.line()) + ": " + what);
  }
}

Matrix3X cube_nodes(Scalar half_extent) {
  Matrix3X nodes(3, 8);
  int column = 0;
  for (Scalar x : {-half_extent, half_extent}) {
    for (Scalar y : {-half_extent, half_extent}) {
      for (Scalar z : {-half_extent, half_extent}) {
        nodes.col(column++) = Vector3 {x, y, z};
      }
    }
  }
  return nodes;
}

std::shared_ptr<BodyModel const> cube_model(std::uint64_t id) {
  std::vector<geometry_config_t> geometries;
  geometries.push_back(dsf_vert_geometry_config_t {
    .properties = geometry_properties_t {
      .id = GeometryId {1},
      .body_from_geometry = pose_t {},
      .material = material_t {.friction = 0.6},
    },
    .nodes = cube_nodes(0.25),
    .sharpness = 60,
  });
  return std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {id},
    .inertial = inertial_t {
      .body_from_inertial = pose_t {},
      .mass = 1.0,
      .inertia = Matrix3::Identity() * 0.1,
    },
    .geometries = std::move(geometries),
  });
}

std::shared_ptr<BodyModel const> ground_model() {
  std::vector<geometry_config_t> geometries;
  geometries.push_back(plane_geometry_config_t {
    .properties = geometry_properties_t {
      .id = GeometryId {9},
      .body_from_geometry = pose_t {},
      .material = material_t {.friction = 0.6},
    },
  });
  return std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {99},
    .inertial = std::nullopt,
    .geometries = std::move(geometries),
  });
}

// Height of the i-th cube in an exactly aligned stack resting on the ground.
Scalar resting_height(int index) {
  return 0.25 + 0.5 * static_cast<Scalar>(index);
}

struct drift_t {
  Scalar position = 0.0;
  Scalar speed = 0.0;
};

// Releases an aligned stack from geometric contact heights. That is slightly
// compressed against the model's own equilibrium -- a DSF cube is a smoothed
// cube and a single one rests at 0.2558 rather than 0.25 -- so the stack
// springs and settles. The transient is deterministic and is what this
// measures.
//
// It is the transient, not the settled state, that reveals solver quality.
// Once settled these stacks drift by microns per second and every solver looks
// alike: measured against the residual change reverted earlier in this branch,
// the settled state was no worse and on some stacks better, while its
// transient was three times too fast. Residuals are no use either -- a solver
// that lets a stack separate has no constraints left to violate, so it reports
// excellent residuals while the stack flies apart.
drift_t settle(int count, int steps) {
  std::vector<BodyInstance> bodies;
  bodies.emplace_back(body_instance_config_t {
    .id = EntityId {100},
    .model = ground_model(),
    .frame_from_body = pose_t {},
    .motion = motion_t {},
    .mobility = mobility_e::static_body,
  });
  for (int i = 0; i < count; ++i) {
    bodies.emplace_back(body_instance_config_t {
      .id = EntityId {static_cast<std::uint64_t>(i + 1)},
      .model = cube_model(static_cast<std::uint64_t>(i + 1)),
      .frame_from_body = pose_t {
        Vector3 {0.0, 0.0, resting_height(i)}, Quaternion::Identity()},
      .motion = motion_t {},
      .mobility = mobility_e::dynamic,
    });
  }
  auto scene = std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
    .frame = FrameId {1},
    .bodies = std::move(bodies),
  });

  Simulator simulator;
  std::shared_ptr<SceneSnapshot const> current = std::move(scene);
  drift_t out;
  for (int step = 0; step < steps; ++step) {
    current = simulator.step(*current, 0.005).snapshot;
    for (int i = 0; i < count; ++i) {
      BodyInstance const& body =
        current->body(EntityId {static_cast<std::uint64_t>(i + 1)});
      out.position = std::max(
        out.position,
        std::abs(body.frameFromBody().position.z() - resting_height(i)));
      out.position = std::max(
        out.position, body.frameFromBody().position.head<2>().norm());
      out.speed = std::max(out.speed, body.motion().linear.norm());
    }
  }
  return out;
}

std::string describe(char const* name, Scalar value, Scalar limit) {
  return std::string {name} + " " + std::to_string(value) + " exceeds " +
    std::to_string(limit);
}

}  // namespace

int main() {
  struct case_t {
    int bodies;
    Scalar position_limit;
    Scalar speed_limit;
  };

  // Ceilings, not reference values: roughly 1.5x what the solver currently
  // produces, so variation between build configurations passes while a real
  // loss of stability fails. They bound the solver's response to the release,
  // not an error against a known-still answer -- the stack is meant to move
  // here, and how fast it moves is the signal.
  //
  // The four-cube stack is the discriminating case. Its peak speed is reached
  // within ten steps and then holds, where an eight-cube stack needs more than
  // a hundred steps to expose the same defect.
  case_t const cases[] = {
    {1, 1.0e-2, 0.35},
    {2, 4.0e-2, 1.05},
    {4, 1.6e-1, 2.50},
  };

  for (case_t const& item : cases) {
    drift_t const drift = settle(item.bodies, 50);
    require(
      std::isfinite(drift.position) && std::isfinite(drift.speed),
      "stack of " + std::to_string(item.bodies) + " diverged");
    require(
      drift.position < item.position_limit,
      "stack of " + std::to_string(item.bodies) + ": " +
        describe("drift", drift.position, item.position_limit));
    require(
      drift.speed < item.speed_limit,
      "stack of " + std::to_string(item.bodies) + ": " +
        describe("speed", drift.speed, item.speed_limit));
  }
}
