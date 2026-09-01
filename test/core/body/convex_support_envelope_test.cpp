#include <stacking_core/body/convex_support_envelope.hpp>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace {

using stacking_core::Vector3;

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("convex support envelope test requirement failed");
  }
}

bool near(Vector3 const& lhs, Vector3 const& rhs) {
  return (lhs - rhs).norm() < 1e-12;
}

stacking_core::geometry_config_t dsf_config(
  stacking_core::GeometryId id, Vector3 const& body_position) {
  using namespace stacking_core;
  Matrix3X nodes(3, 6);
  nodes.col(0) = Vector3::UnitX();
  nodes.col(1) = -Vector3::UnitX();
  nodes.col(2) = Vector3::UnitY();
  nodes.col(3) = -Vector3::UnitY();
  nodes.col(4) = Vector3::UnitZ();
  nodes.col(5) = -Vector3::UnitZ();
  return dsf_vert_geometry_config_t {
    .properties = {
      .id = id,
      .body_from_geometry =
        pose_t {body_position, Quaternion::Identity()},
      .material = material_t {},
    },
    .nodes = std::move(nodes),
  };
}

}  // namespace

int main() {
  using namespace stacking_core;

  BodyModel body {{
    .id = BodyModelId {10},
    .inertial = std::nullopt,
    .geometries = {
      dsf_config(GeometryId {1}, Vector3 {1.0, 0.0, 0.0}),
      dsf_config(GeometryId {2}, Vector3 {3.0, 0.0, 0.0}),
      point_geometry_config_t {
        .properties = {
          .id = GeometryId {3},
          .body_from_geometry = pose_t {},
          .material = material_t {},
        },
      },
    },
  }};

  ConvexSupportEnvelope const envelope {body};
  require(envelope.bodyModelId() == BodyModelId {10});
  require(envelope.geometryCount() == 2);

  pose_t const world_from_body {
    Vector3 {10.0, 0.0, 0.0}, Quaternion::Identity()};
  envelope_support_t const positive =
    envelope.support(Vector3::UnitX(), world_from_body);
  require(std::abs(positive.h - 14.0) < 1e-12);
  require(near(positive.s, Vector3 {14.0, 0.0, 0.0}));
  require(positive.source_geometry == GeometryId {2});
  require(!positive.has_tie);

  envelope_support_t const negative =
    envelope.support(-Vector3::UnitX(), pose_t {});
  require(std::abs(negative.h) < 1e-12);
  require(near(negative.s, Vector3::Zero()));
  require(negative.source_geometry == GeometryId {1});
  require(!negative.has_tie);

  Quaternion const quarter_turn {
    Eigen::AngleAxisd(std::numbers::pi / 2.0, Vector3::UnitZ())};
  envelope_support_t const rotated = envelope.support(
    Vector3::UnitY(), pose_t {Vector3::Zero(), quarter_turn});
  require(std::abs(rotated.h - 4.0) < 1e-12);
  require(near(rotated.s, Vector3 {0.0, 4.0, 0.0}));
  require(rotated.source_geometry == GeometryId {2});

  envelope_support_t const tied =
    envelope.support(Vector3::UnitZ(), pose_t {});
  require(std::abs(tied.h - 1.0) < 1e-12);
  require(tied.source_geometry == GeometryId {1});
  require(tied.has_tie);

  bool rejected_body_without_dsf = false;
  try {
    BodyModel const point_body {{
      .id = BodyModelId {11},
      .inertial = std::nullopt,
      .geometries = {
        point_geometry_config_t {
          .properties = {
            .id = GeometryId {4},
            .body_from_geometry = pose_t {},
            .material = material_t {},
          },
        },
      },
    }};
    ConvexSupportEnvelope const invalid_envelope {point_body};
    (void)invalid_envelope;
  } catch (std::invalid_argument const&) {
    rejected_body_without_dsf = true;
  }
  require(rejected_body_without_dsf);
}
