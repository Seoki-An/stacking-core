#include <stacking_core/body/model.hpp>

#include <stdexcept>
#include <utility>

namespace {

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("body model test requirement failed");
  }
}

stacking_core::geometry_config_t point_config(stacking_core::GeometryId id) {
  using namespace stacking_core;
  return point_geometry_config_t {
    .properties = {
      .id = id,
      .body_from_geometry = pose_t {},
      .material = material_t {},
    },
  };
}

}  // namespace

int main() {
  using namespace stacking_core;

  std::vector<geometry_config_t> geometries;
  geometries.push_back(point_config(GeometryId {1}));
  geometries.push_back(plane_geometry_config_t {
    .properties = {
      .id = GeometryId {2},
      .body_from_geometry = pose_t {},
      .material = material_t {.friction = 0.5},
    },
  });

  BodyModel model {{
    .id = BodyModelId {10},
    .inertial = inertial_t {
      .body_from_inertial = pose_t {},
      .mass = 2.0,
      .inertia = 3.0 * Matrix3::Identity(),
    },
    .geometries = std::move(geometries),
  }};
  require(model.id() == BodyModelId {10});
  require(model.inertial().mass == 2.0);
  require(model.geometryCount() == 2);
  require(model.geometry(0).type() == geometry_type_e::point);
  require(model.geometry(GeometryId {2}).type() == geometry_type_e::plane);
  require(model.findGeometry(GeometryId {3}) == nullptr);

  BodyModel const empty_model {{
    .id = BodyModelId {11},
    .inertial = std::nullopt,
    .geometries = {},
  }};
  require(!empty_model.hasInertial());
  require(empty_model.geometryCount() == 0);
  require(empty_model.boundingVolume().isEmpty());

  bool rejected_duplicate = false;
  try {
    std::vector<geometry_config_t> duplicates;
    duplicates.push_back(point_config(GeometryId {4}));
    duplicates.push_back(point_config(GeometryId {4}));
    BodyModel const invalid_model {{
      .id = BodyModelId {12},
      .inertial = inertial_t {},
      .geometries = std::move(duplicates),
    }};
    (void)invalid_model;
  } catch (std::invalid_argument const&) {
    rejected_duplicate = true;
  }
  require(rejected_duplicate);

  bool rejected_inertia = false;
  try {
    BodyModel const invalid_model {{
      .id = BodyModelId {13},
      .inertial = inertial_t {
        .body_from_inertial = pose_t {},
        .mass = 1.0,
        .inertia = Matrix3::Zero(),
      },
      .geometries = {point_config(GeometryId {5})},
    }};
    (void)invalid_model;
  } catch (std::invalid_argument const&) {
    rejected_inertia = true;
  }
  require(rejected_inertia);
}
