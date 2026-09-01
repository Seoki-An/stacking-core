#include <stacking_core/contact.hpp>

#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
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
      "contact test requirement failed at line " +
      std::to_string(location.line()));
  }
}

bool near(stacking_core::Scalar first, stacking_core::Scalar second) {
  return std::abs(first - second) <= 1e-10;
}

std::shared_ptr<stacking_core::BodyModel const> point_model(
  stacking_core::BodyModelId model_id,
  stacking_core::GeometryId geometry_id,
  stacking_core::Scalar friction) {
  using namespace stacking_core;
  return std::make_shared<BodyModel>(body_model_config_t {
    .id = model_id,
    .inertial = std::nullopt,
    .geometries = {
      point_geometry_config_t {
        .properties = {
          .id = geometry_id,
          .body_from_geometry = pose_t {},
          .material = material_t {.friction = friction},
        },
      },
    },
  });
}

stacking_core::BodyInstance point_body(
  stacking_core::EntityId entity,
  std::shared_ptr<stacking_core::BodyModel const> model) {
  using namespace stacking_core;
  return BodyInstance {body_instance_config_t {
    .id = entity,
    .model = std::move(model),
    .frame_from_body = pose_t {},
    .motion = motion_t {},
    .mobility = mobility_e::dynamic,
  }};
}

}  // namespace

int main() {
  using namespace stacking_core;

  contact_frame_t const frame = make_contact_frame(Vector3 {1.0, 2.0, 3.0});
  require(near(frame.normal.norm(), 1.0));
  require(near(frame.tangent_first.norm(), 1.0));
  require(near(frame.tangent_second.norm(), 1.0));
  require(near(frame.normal.dot(frame.tangent_first), 0.0));
  require(near(frame.normal.dot(frame.tangent_second), 0.0));
  require(frame.tangent_first.cross(frame.tangent_second).isApprox(frame.normal));
  require(frame.frameFromContact().determinant() > 0.0);

  require(near(combine_friction(1.0, 3.0), 1.5));
  require(near(combine_friction(0.0, 0.0), 0.0));

  std::vector<BodyInstance> bodies;
  bodies.push_back(point_body(
    EntityId {1}, point_model(BodyModelId {1}, GeometryId {1}, 1.0)));
  bodies.push_back(point_body(
    EntityId {2}, point_model(BodyModelId {2}, GeometryId {2}, 3.0)));
  SceneSnapshot const scene {scene_snapshot_config_t {
    .frame = FrameId {1},
    .bodies = std::move(bodies),
  }};
  contact_t const contact = make_contact(scene, contact_feature_t {
    .pair = collision_pair_t {
      .first = geometry_instance_id_t {
        .entity = EntityId {1}, .geometry = GeometryId {1}},
      .second = geometry_instance_id_t {
        .entity = EntityId {2}, .geometry = GeometryId {2}},
    },
    .gap = -0.1,
    .point_first = Vector3::Zero(),
    .point_second = Vector3 {0.0, 0.0, -0.1},
    .normal = 2.0 * Vector3::UnitZ(),
  });
  require(near(contact.friction, 1.5));
  require(contact.feature.normal.isApprox(Vector3::UnitZ()));

  require(project_coulomb_impulse(
    Vector3 {1.0, 2.0, -1.0}, 0.5).isZero());
  require(project_coulomb_impulse(
    Vector3 {0.3, 0.4, 1.0}, 0.5)
      .isApprox(Vector3 {0.3, 0.4, 1.0}));
  require(project_coulomb_impulse(
    Vector3 {3.0, 4.0, 2.0}, 0.5)
      .isApprox(Vector3 {0.6, 0.8, 2.0}));
  require(project_coulomb_impulse(
    Vector3 {3.0, 4.0, 2.0}, 0.0)
      .isApprox(Vector3 {0.0, 0.0, 2.0}));

  Scalar const circle_torsion = torsional_friction_coefficient(
    0.6, elliptical_contact_patch_t {
      .semi_axis_first = 2.0,
      .semi_axis_second = 2.0,
    });
  require(near(circle_torsion, 0.8));
  Scalar const line_torsion = torsional_friction_coefficient(
    0.6, elliptical_contact_patch_t {
      .semi_axis_first = 0.0,
      .semi_axis_second = 2.0,
    });
  require(near(line_torsion, 0.6 * 8.0 / (3.0 * std::numbers::pi)));

  require(project_limit_surface_impulse(
    Vector4 {1.0, 2.0, -1.0, 3.0}, 0.5, 0.2).isZero());
  require(project_limit_surface_impulse(
    Vector4 {0.2, 0.0, 1.0, 0.1}, 0.5, 0.2)
      .isApprox(Vector4 {0.2, 0.0, 1.0, 0.1}));
  require(project_limit_surface_impulse(
    Vector4 {2.0, 0.0, 1.0, 0.0}, 0.5, 0.2)
      .isApprox(Vector4 {0.5, 0.0, 1.0, 0.0}));
  require(project_limit_surface_impulse(
    Vector4 {0.0, 0.0, 1.0, 2.0}, 0.5, 0.2)
      .isApprox(Vector4 {0.0, 0.0, 1.0, 0.2}));

  Scalar const diagonal = 0.5 / std::sqrt(2.0);
  Vector4 const mixed = project_limit_surface_impulse(
    Vector4 {1.0, 0.0, 1.0, 1.0}, 0.5, 0.5);
  require(mixed.isApprox(Vector4 {diagonal, 0.0, 1.0, diagonal}, 1e-10));
  require(project_limit_surface_impulse(
    Vector4 {1.0, 0.0, 1.0, 1.0}, 0.0, 0.2)
      .isApprox(Vector4 {0.0, 0.0, 1.0, 0.2}));
  require(project_limit_surface_impulse(
    Vector4 {1.0, 0.0, 1.0, 1.0}, 0.5, 0.0)
      .isApprox(Vector4 {0.5, 0.0, 1.0, 0.0}));

  bool rejected_zero_normal = false;
  try {
    (void)make_contact_frame(Vector3::Zero());
  } catch (std::invalid_argument const&) {
    rejected_zero_normal = true;
  }
  require(rejected_zero_normal);

  bool rejected_negative_friction = false;
  try {
    (void)project_coulomb_impulse(Vector3::UnitZ(), -1.0);
  } catch (std::invalid_argument const&) {
    rejected_negative_friction = true;
  }
  require(rejected_negative_friction);

  bool rejected_nonfinite_impulse = false;
  try {
    Vector4 impulse = Vector4::Zero();
    impulse.x() = std::numeric_limits<Scalar>::infinity();
    (void)project_limit_surface_impulse(impulse, 0.5, 0.2);
  } catch (std::invalid_argument const&) {
    rejected_nonfinite_impulse = true;
  }
  require(rejected_nonfinite_impulse);
}
