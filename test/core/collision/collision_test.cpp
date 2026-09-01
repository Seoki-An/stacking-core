#include <stacking_core/collision.hpp>

#include <cmath>
#include <memory>
#include <source_location>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

  void require(
    bool condition,
    std::source_location location = std::source_location::current()) {
    if (!condition) {
      throw std::runtime_error(
        "collision test requirement failed at line " +
        std::to_string(location.line()));
    }
  }

  bool near(stacking_core::Scalar first, stacking_core::Scalar second) {
    return std::abs(first - second) <= 1e-8;
  }

  std::shared_ptr<stacking_core::BodyModel const> make_plane_model() {
    using namespace stacking_core;
    std::vector<geometry_config_t> geometries;
    geometries.push_back(plane_geometry_config_t {
      .properties =
        geometry_properties_t {
          .id = GeometryId {1},
          .body_from_geometry = pose_t {},
          .material = material_t {},
        },
    });
    geometries.push_back(plane_geometry_config_t {
      .properties =
        geometry_properties_t {
          .id = GeometryId {4},
          .body_from_geometry =
            pose_t {Vector3 {0.0, 0.0, -10.0}, Quaternion::Identity()},
          .material = material_t {},
        },
    });
    return std::make_shared<BodyModel>(body_model_config_t {
      .id = BodyModelId {1},
      .inertial = std::nullopt,
      .geometries = std::move(geometries),
    });
  }

  std::shared_ptr<stacking_core::BodyModel const> make_point_model() {
    using namespace stacking_core;
    std::vector<geometry_config_t> geometries;
    geometries.push_back(point_geometry_config_t {
      .properties =
        geometry_properties_t {
          .id = GeometryId {3},
          .body_from_geometry = pose_t {},
          .material = material_t {},
        },
    });
    return std::make_shared<BodyModel>(body_model_config_t {
      .id = BodyModelId {3},
      .inertial = std::nullopt,
      .geometries = std::move(geometries),
    });
  }

  std::shared_ptr<stacking_core::BodyModel const> make_cube_model(
    stacking_core::BodyModelId model_id,
    stacking_core::GeometryId geometry_id) {
    using namespace stacking_core;
    Matrix3X nodes(3, 8);
    nodes << -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0,
      -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0;
    std::vector<geometry_config_t> geometries;
    geometries.push_back(dsf_vert_geometry_config_t {
      .properties =
        geometry_properties_t {
          .id = geometry_id,
          .body_from_geometry = pose_t {},
          .material = material_t {},
        },
      .nodes = std::move(nodes),
      .sharpness = 20,
    });
    return std::make_shared<BodyModel>(body_model_config_t {
      .id = model_id,
      .inertial = std::nullopt,
      .geometries = std::move(geometries),
    });
  }

  std::shared_ptr<stacking_core::BodyModel const> make_anisotropic_model() {
    using namespace stacking_core;
    Matrix3X nodes(3, 8);
    nodes << -2.0, -2.0, -2.0, -2.0, 2.0, 2.0, 2.0, 2.0, -0.7, -0.7, 0.7, 0.7,
      -0.7, -0.7, 0.7, 0.7, -0.4, 0.4, -0.4, 0.4, -0.4, 0.4, -0.4, 0.4;
    std::vector<geometry_config_t> geometries;
    geometries.push_back(dsf_vert_geometry_config_t {
      .properties =
        geometry_properties_t {
          .id = GeometryId {30},
          .body_from_geometry = pose_t {},
          .material = material_t {},
        },
      .nodes = std::move(nodes),
      .sharpness = 20,
    });
    return std::make_shared<BodyModel>(body_model_config_t {
      .id = BodyModelId {30},
      .inertial = std::nullopt,
      .geometries = std::move(geometries),
    });
  }

  stacking_core::BodyInstance make_body(
    stacking_core::EntityId entity,
    std::shared_ptr<stacking_core::BodyModel const> model,
    stacking_core::Vector3 position, stacking_core::mobility_e mobility) {
    using namespace stacking_core;
    return BodyInstance {body_instance_config_t {
      .id = entity,
      .model = std::move(model),
      .frame_from_body = pose_t {position, Quaternion::Identity()},
      .motion = motion_t {},
      .mobility = mobility,
    }};
  }

  std::shared_ptr<stacking_core::SceneSnapshot const> make_main_scene() {
    using namespace stacking_core;
    std::vector<BodyInstance> bodies;
    bodies.push_back(make_body(
      EntityId {1}, make_plane_model(), Vector3::Zero(),
      mobility_e::static_body));
    bodies.push_back(make_body(
      EntityId {2}, make_cube_model(BodyModelId {2}, GeometryId {2}),
      Vector3 {0.0, 0.0, 2.0}, mobility_e::dynamic));
    bodies.push_back(make_body(
      EntityId {3}, make_point_model(), Vector3 {0.0, 0.0, 4.0},
      mobility_e::kinematic));
    return std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
      .frame = FrameId {1},
      .bodies = std::move(bodies),
    });
  }

  std::shared_ptr<stacking_core::SceneSnapshot const> make_cube_pair_scene(
    stacking_core::Scalar second_x) {
    using namespace stacking_core;
    std::shared_ptr<BodyModel const> const cube =
      make_cube_model(BodyModelId {5}, GeometryId {5});
    std::vector<BodyInstance> bodies;
    bodies.push_back(
      make_body(EntityId {10}, cube, Vector3::Zero(), mobility_e::kinematic));
    bodies.push_back(make_body(
      EntityId {11}, cube, Vector3 {second_x, 0.0, 0.0},
      mobility_e::kinematic));
    return std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
      .frame = FrameId {2},
      .bodies = std::move(bodies),
    });
  }

  std::shared_ptr<stacking_core::SceneSnapshot const> make_plane_cube_scene(
    stacking_core::Scalar cube_z) {
    using namespace stacking_core;
    std::vector<BodyInstance> bodies;
    bodies.push_back(make_body(
      EntityId {20}, make_plane_model(), Vector3::Zero(),
      mobility_e::static_body));
    bodies.push_back(make_body(
      EntityId {21}, make_cube_model(BodyModelId {21}, GeometryId {21}),
      Vector3 {0.0, 0.0, cube_z}, mobility_e::dynamic));
    return std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
      .frame = FrameId {3},
      .bodies = std::move(bodies),
    });
  }

  std::shared_ptr<stacking_core::SceneSnapshot const>
  make_rotated_anisotropic_pair_scene(
    stacking_core::Vector3 second_offset = stacking_core::Vector3::Zero()) {
    using namespace stacking_core;
    std::shared_ptr<BodyModel const> const model = make_anisotropic_model();
    Quaternion const first_orientation {
      Eigen::AngleAxisd(0.5, Vector3 {1.0, 2.0, 0.5}.normalized())};
    Quaternion const second_orientation {
      Eigen::AngleAxisd(-0.7, Vector3 {-1.0, 0.5, 2.0}.normalized())};
    std::vector<BodyInstance> bodies;
    bodies.emplace_back(body_instance_config_t {
      .id = EntityId {30},
      .model = model,
      .frame_from_body = pose_t {Vector3::Zero(), first_orientation},
      .motion = motion_t {},
      .mobility = mobility_e::kinematic,
    });
    bodies.emplace_back(body_instance_config_t {
      .id = EntityId {31},
      .model = model,
      .frame_from_body =
        pose_t {Vector3 {3.1, 1.2, 0.6} + second_offset, second_orientation},
      .motion = motion_t {},
      .mobility = mobility_e::kinematic,
    });
    return std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
      .frame = FrameId {4},
      .bodies = std::move(bodies),
    });
  }

}  // namespace

int main() {
  using namespace stacking_core;

  std::shared_ptr<SceneSnapshot const> const snapshot = make_main_scene();
  SceneView const view {snapshot, {EntityId {1}, EntityId {2}, EntityId {3}}};
  std::vector<collision_body_pair_t> const body_pairs =
    brute_force_broad_phase(view);
  require(body_pairs.size() == 3);
  require(body_pairs[0].first == EntityId {1});
  require(body_pairs[0].second == EntityId {2});

  std::vector<collision_pair_t> const geometry_pairs =
    brute_force_middle_phase(view, body_pairs);
  require(geometry_pairs.size() == 5);
  require(geometry_pairs[0].first.geometry == GeometryId {1});
  require(geometry_pairs[1].first.geometry == GeometryId {4});
  require(
    !supports_collision_pair(geometry_type_e::plane, geometry_type_e::plane));

  Scalar const smooth_radius = std::pow(4.0, 1.0 / 20.0);
  BoundingVolume const& cube_bounds = snapshot->body(EntityId {2})
                                        .model()
                                        .geometry(GeometryId {2})
                                        .boundingVolume();
  require(cube_bounds.is_finite());
  require(cube_bounds.minimum().isApprox(-smooth_radius * Vector3::Ones()));
  require(cube_bounds.maximum().isApprox(smooth_radius * Vector3::Ones()));
  require(snapshot->body(EntityId {1}).model().boundingVolume().isUnbounded());

  std::vector<collision_body_pair_t> const bounded_body_pairs =
    bounding_volume_broad_phase(view);
  require(bounded_body_pairs.size() == 2);
  std::vector<collision_pair_t> const bounded_geometry_pairs =
    bounding_volume_middle_phase(view, bounded_body_pairs);
  require(bounded_geometry_pairs.empty());

  contact_feature_t const plane_dsf =
    compute_contact(*snapshot, geometry_pairs[0]);
  require(near(plane_dsf.gap, 2.0 - smooth_radius));
  require(plane_dsf.normal.isApprox(Vector3::UnitZ()));
  require(near(plane_dsf.point_first.z(), 0.0));
  require(near(plane_dsf.point_second.z(), 2.0 - smooth_radius));
  require(!plane_dsf.penetrating());

  collision_pair_t const reversed_pair {
    .first = geometry_pairs[0].second,
    .second = geometry_pairs[0].first,
  };
  contact_feature_t const reversed_feature =
    compute_contact(*snapshot, reversed_pair);
  require(near(reversed_feature.gap, plane_dsf.gap));
  require(reversed_feature.normal.isApprox(-plane_dsf.normal));
  require(reversed_feature.point_first.isApprox(plane_dsf.point_second));

  collision_pair_t const plane_point_pair {
    .first =
      geometry_instance_id_t {
        .entity = EntityId {1}, .geometry = GeometryId {1}},
    .second =
      geometry_instance_id_t {
        .entity = EntityId {3}, .geometry = GeometryId {3}},
  };
  contact_feature_t const plane_point =
    compute_contact(*snapshot, plane_point_pair);
  require(near(plane_point.gap, 4.0));

  collision_pair_t const dsf_point_pair {
    .first =
      geometry_instance_id_t {
        .entity = EntityId {2}, .geometry = GeometryId {2}},
    .second =
      geometry_instance_id_t {
        .entity = EntityId {3}, .geometry = GeometryId {3}},
  };
  contact_feature_t const dsf_point =
    compute_contact(*snapshot, dsf_point_pair);
  require(near(dsf_point.gap, 2.0 - smooth_radius));
  require(dsf_point.normal.isApprox(Vector3::UnitZ()));

  std::shared_ptr<SceneSnapshot const> const separated =
    make_cube_pair_scene(3.0);
  collision_pair_t const cube_pair {
    .first =
      geometry_instance_id_t {
        .entity = EntityId {10}, .geometry = GeometryId {5}},
    .second =
      geometry_instance_id_t {
        .entity = EntityId {11}, .geometry = GeometryId {5}},
  };
  contact_feature_t const separated_feature =
    compute_contact(*separated, cube_pair);
  require(near(separated_feature.gap, 3.0 - 2.0 * smooth_radius));
  require(separated_feature.normal.isApprox(Vector3::UnitX()));
  SceneView const separated_view {separated, {EntityId {10}, EntityId {11}}};
  require(bounding_volume_broad_phase(separated_view).empty());
  std::vector<collision_body_pair_t> const separated_margin_pairs =
    bounding_volume_broad_phase(separated_view, 1.0);
  require(separated_margin_pairs.size() == 1);
  require(
    bounding_volume_middle_phase(separated_view, separated_margin_pairs, 1.0)
      .size() == 1);

  std::shared_ptr<SceneSnapshot const> const penetrating =
    make_cube_pair_scene(1.5);
  contact_feature_t const penetrating_feature =
    compute_contact(*penetrating, cube_pair);
  require(near(penetrating_feature.gap, 1.5 - 2.0 * smooth_radius));
  require(penetrating_feature.penetrating());
  SceneView const penetrating_view {
    penetrating, {EntityId {10}, EntityId {11}}};
  std::vector<collision_body_pair_t> const penetrating_body_pairs =
    bounding_volume_broad_phase(penetrating_view);
  require(penetrating_body_pairs.size() == 1);
  require(
    bounding_volume_middle_phase(penetrating_view, penetrating_body_pairs)
      .size() == 1);

  std::shared_ptr<SceneSnapshot const> const anisotropic =
    make_rotated_anisotropic_pair_scene();
  collision_pair_t const anisotropic_pair {
    .first =
      geometry_instance_id_t {
        .entity = EntityId {30}, .geometry = GeometryId {30}},
    .second =
      geometry_instance_id_t {
        .entity = EntityId {31}, .geometry = GeometryId {30}},
  };
  auto const& anisotropic_geometry = static_cast<DsfVertGeometry const&>(
    anisotropic->body(EntityId {30}).model().geometry(GeometryId {30}));
  Vector3 const initial_direction = Vector3 {3.1, 1.2, 0.6}.normalized();
  support_t const initial_first = anisotropic_geometry.support(
    initial_direction, anisotropic->body(EntityId {30}).frameFromBody());
  support_t const initial_second = anisotropic_geometry.support(
    -initial_direction, anisotropic->body(EntityId {31}).frameFromBody());
  Vector3 const initial_gradient = initial_first.s - initial_second.s;
  Vector3 const initial_tangent_gradient = initial_gradient -
    initial_gradient.dot(initial_direction) * initial_direction;
  require(initial_tangent_gradient.norm() > 1e-3);

  contact_feature_t const anisotropic_feature =
    compute_contact(*anisotropic, anisotropic_pair);
  require(near(anisotropic_feature.normal.norm(), 1.0));
  Vector3 const converged_gradient =
    anisotropic_feature.point_first - anisotropic_feature.point_second;
  Vector3 const converged_tangent_gradient = converged_gradient -
    converged_gradient.dot(anisotropic_feature.normal) *
      anisotropic_feature.normal;
  require(converged_tangent_gradient.norm() < 2e-6);

  diffable_contact_feature_t const diffable_feature =
    compute_diffable_contact(*anisotropic, anisotropic_pair);
  constexpr Scalar derivative_step = 1e-6;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    Vector3 offset = Vector3::Zero();
    offset[axis] = derivative_step;
    contact_feature_t const plus = compute_contact(
      *make_rotated_anisotropic_pair_scene(offset), anisotropic_pair);
    contact_feature_t const minus = compute_contact(
      *make_rotated_anisotropic_pair_scene(-offset), anisotropic_pair);
    Scalar const numeric_gap = (plus.gap - minus.gap) / (2.0 * derivative_step);
    Vector3 const numeric_normal =
      (plus.normal - minus.normal) / (2.0 * derivative_step);
    require(std::abs(diffable_feature.d_gap[6 + axis] - numeric_gap) < 2e-5);
    require(
      (diffable_feature.d_normal.col(6 + axis) - numeric_normal).norm() < 2e-5);
  }

  std::shared_ptr<SceneSnapshot const> const plane_penetrating =
    make_plane_cube_scene(0.5);
  SceneView const plane_penetrating_view {
    plane_penetrating, {EntityId {20}, EntityId {21}}};
  std::vector<collision_body_pair_t> const plane_body_pairs =
    bounding_volume_broad_phase(plane_penetrating_view);
  require(plane_body_pairs.size() == 1);
  std::vector<collision_pair_t> const plane_geometry_pairs =
    bounding_volume_middle_phase(plane_penetrating_view, plane_body_pairs);
  require(plane_geometry_pairs.size() == 1);
  require(plane_geometry_pairs[0].first.geometry == GeometryId {1});

  diffable_contact_feature_t const plane_diffable =
    compute_diffable_contact(*plane_penetrating, plane_geometry_pairs[0]);
  constexpr Scalar plane_derivative_step = 1e-6;
  contact_feature_t const plane_plus = compute_contact(
    *make_plane_cube_scene(0.5 + plane_derivative_step),
    plane_geometry_pairs[0]);
  contact_feature_t const plane_minus = compute_contact(
    *make_plane_cube_scene(0.5 - plane_derivative_step),
    plane_geometry_pairs[0]);
  Scalar const plane_numeric_gap =
    (plane_plus.gap - plane_minus.gap) / (2.0 * plane_derivative_step);
  require(std::abs(plane_diffable.d_gap[8] - plane_numeric_gap) < 1e-8);
  require(plane_diffable.d_normal.col(8).norm() < 1e-12);

  collision_pair_t const plane_reversed_pair {
    .first = plane_geometry_pairs[0].second,
    .second = plane_geometry_pairs[0].first,
  };
  diffable_contact_feature_t const plane_reversed =
    compute_diffable_contact(*plane_penetrating, plane_reversed_pair);
  require(near(plane_reversed.gap, plane_diffable.gap));
  require(plane_reversed.normal.isApprox(-plane_diffable.normal));
  require(std::abs(plane_reversed.d_gap[2] - plane_numeric_gap) < 1e-8);

  bool rejected_same_body = false;
  try {
    std::vector<collision_body_pair_t> const invalid_pairs = {
      collision_body_pair_t {.first = EntityId {1}, .second = EntityId {1}},
    };
    std::vector<collision_pair_t> const invalid =
      brute_force_middle_phase(view, invalid_pairs);
    (void)invalid;
  } catch (std::invalid_argument const&) {
    rejected_same_body = true;
  }
  require(rejected_same_body);
}
