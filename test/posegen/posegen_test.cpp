#include <stacking_core/posegen.hpp>

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
  std::source_location location = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error(
      "posegen test requirement failed at line " +
      std::to_string(location.line()));
  }
}

Matrix3X cube_nodes(Vector3 const& center, Vector3 const& half_extent) {
  Matrix3X nodes(3, 8);
  int col = 0;
  for (Scalar x : {-half_extent.x(), half_extent.x()}) {
    for (Scalar y : {-half_extent.y(), half_extent.y()}) {
      for (Scalar z : {-half_extent.z(), half_extent.z()}) {
        nodes.col(col++) = center + Vector3 {x, y, z};
      }
    }
  }
  return nodes;
}

std::shared_ptr<BodyModel const> make_model(
  std::uint64_t model_id,
  std::vector<Matrix3X> nodes) {
  std::vector<geometry_config_t> geometries;
  std::uint64_t geometry_id = 1;
  for (Matrix3X& geometry_nodes : nodes) {
    geometries.push_back(dsf_vert_geometry_config_t {
      .properties = geometry_properties_t {
        .id = GeometryId {geometry_id++},
        .body_from_geometry = pose_t {},
        .material = material_t {.friction = 0.6},
      },
      .nodes = std::move(geometry_nodes),
      .sharpness = 70,
    });
  }
  return std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {model_id},
    .inertial = inertial_t {
      .body_from_inertial = pose_t {},
      .mass = 1.0,
      .inertia = Matrix3::Identity(),
    },
    .geometries = std::move(geometries),
  });
}

BodyInstance make_body(
  std::uint64_t id,
  std::shared_ptr<BodyModel const> model,
  Vector3 const& position) {
  return BodyInstance(body_instance_config_t {
    .id = EntityId {id},
    .model = std::move(model),
    .frame_from_body = pose_t {position, Quaternion::Identity()},
    .motion = motion_t {},
    .mobility = mobility_e::dynamic,
  });
}

struct scene_t {
  std::shared_ptr<SceneSnapshot const> snapshot;
  SceneView view;
};

scene_t make_scene(std::vector<BodyInstance> bodies) {
  std::vector<EntityId> ids;
  ids.reserve(bodies.size());
  for (BodyInstance const& body : bodies) {
    ids.push_back(body.id());
  }
  auto snapshot = std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
    .frame = FrameId {1},
    .bodies = std::move(bodies),
  });
  SceneView view {snapshot, std::move(ids)};
  return scene_t {std::move(snapshot), std::move(view)};
}

void require_pose_near(
  pose_t const& actual,
  Vector3 const& expected_position,
  Quaternion const& expected_orientation,
  Scalar tol) {
  require((actual.position - expected_position).norm() < tol);
  require(actual.orientation.angularDistance(expected_orientation) < tol);
}

void require_solver_stats(
  posegen_result_t const& result,
  posegen_config_t const& config) {
  posegen_solver_stats_t const& solver = result.solver;
  require(solver.iters >= 0);
  require(solver.iters <= config.trust_region.max_iters);
  require(solver.accepted_iters <= solver.iters);
  require(solver.objective_evals == solver.iters + 1);
  require(
    solver.scene_graph_rebuilds + solver.scene_graph_reuses ==
    solver.objective_evals);
  require(std::isfinite(solver.grad_norm));
  require(std::isfinite(solver.trust_region_radius));
  require(solver.force_solver.iters > 0);
  require(solver.force_solver.iters <= config.force_solver.max_iters);
  require(std::isfinite(solver.force_solver.primal_residual));
  require(std::isfinite(solver.force_solver.dual_residual));
}

}  // namespace

int main() {
  using namespace stacking_core;

  auto const cube = make_model(1, {
    cube_nodes(Vector3::Zero(), Vector3::Constant(0.5)),
  });
  auto const target = make_model(2, {
    cube_nodes(
      Vector3 {0.0, 0.0, 0.75}, Vector3 {0.8, 0.8, 0.75}),
  });
  auto const two_cube = make_model(3, {
    cube_nodes(
      Vector3 {-0.3, 0.0, 0.0}, Vector3 {0.25, 0.25, 0.5}),
    cube_nodes(
      Vector3 {0.3, 0.0, 0.0}, Vector3 {0.25, 0.25, 0.5}),
  });

  PoseGenerator generator;
  posegen_config_t const config = generator.config();

  scene_t ground_scene = make_scene({
    make_body(1, cube, Vector3 {0.0, 0.0, 1.5}),
  });
  posegen_problem_t const ground_problem {
    .scene = ground_scene.view,
    .candidate = EntityId {1},
    .targets = {},
    .boundaries = {},
  };
  posegen_result_t const ground = generator.solve(ground_problem);
  require(is_valid(ground.optimal_pose));
  // Emitted by diffsim's DSF posegen for the identical cube and defaults.
  require_pose_near(
    ground.optimal_pose,
    Vector3 {0.0, 0.0, 0.40737580471059948},
    Quaternion::Identity(),
    2e-12);
  require(std::abs(ground.c_feq - 0.0047708433048919497) < 2e-12);
  require(std::abs(ground.c_gap - 0.10262500000000002) < 2e-12);
  require(ground.candidate_contact_forces.cols() == 1);
  require(ground.net_wrench.contains(EntityId {1}));
  require(ground.solver.scene_graph_rebuilds == 1);
  require_solver_stats(ground, config);

  scene_t continued_ground_scene = make_scene({
    make_body(1, cube, ground.optimal_pose.position),
  });
  posegen_result_t const continued_ground = generator.solve(
    posegen_problem_t {
      .scene = continued_ground_scene.view,
      .candidate = EntityId {1},
      .targets = {},
      .boundaries = {},
    });
  require(continued_ground.solver.scene_graph_rebuilds == 0);
  require(
    continued_ground.solver.scene_graph_reuses ==
    continued_ground.solver.objective_evals);
  require_solver_stats(continued_ground, config);

  scene_t stack_scene = make_scene({
    make_body(1, cube, Vector3 {0.1, 0.0, 1.4}),
    make_body(2, cube, Vector3 {0.0, 0.0, 0.5}),
  });
  posegen_result_t const stack = generator.solve(posegen_problem_t {
    .scene = stack_scene.view,
    .candidate = EntityId {1},
    .targets = {},
    .boundaries = {},
  });
  require_pose_near(
    stack.optimal_pose,
    Vector3 {0.1001974763390623, 0.0, 1.4173060772672321},
    Quaternion {
      0.99999999996657574, 0.0, 8.1760890823788104e-6, 0.0},
    5e-7);
  require(std::abs(stack.c_feq - 0.0094316596616706716) < 1e-9);
  require(std::abs(stack.c_gap - 0.10262317984041754) < 1e-9);
  require_solver_stats(stack, config);

  scene_t target_scene = make_scene({
    make_body(1, cube, Vector3 {1.2, 0.0, 1.3}),
    make_body(3, target, Vector3::Zero()),
  });
  posegen_result_t const target_result = generator.solve(posegen_problem_t {
    .scene = target_scene.view,
    .candidate = EntityId {1},
    .targets = {EntityId {3}},
    .boundaries = {},
  });
  require_pose_near(
    target_result.optimal_pose,
    Vector3 {
      0.83357780678175786,
      3.7833720132819969e-7,
      0.40745566664618427},
    Quaternion {
      0.99999999999999611,
      -1.3128509616297982e-11,
      8.7108734259999017e-8,
      -6.7547516551584151e-9},
    3e-6);
  require(std::abs(target_result.c_feq - 0.0047169948256394357) < 1e-9);
  require(std::abs(target_result.c_gap - 0.10254513806494148) < 1e-8);
  require(target_result.optimal_pose.position.x() < 1.0);
  require_solver_stats(target_result, config);

  scene_t multi_scene = make_scene({
    make_body(1, two_cube, Vector3 {0.0, 0.0, 1.4}),
    make_body(2, cube, Vector3 {0.0, 0.0, 0.5}),
  });
  posegen_result_t const multi = generator.solve(posegen_problem_t {
    .scene = multi_scene.view,
    .candidate = EntityId {1},
    .targets = {},
    .boundaries = {},
  });
  require_pose_near(
    multi.optimal_pose,
    Vector3 {0.0, 0.0, 1.4776279677916129},
    Quaternion::Identity(),
    1e-8);
  require(std::abs(multi.c_feq - 0.0058921289971417849) < 1e-10);
  require(std::abs(multi.c_gap - 0.082628386011444011) < 1e-9);
  require(multi.candidate_contact_forces.cols() == 2);
  require_solver_stats(multi, config);

  scene_t boundary_scene = make_scene({
    make_body(1, cube, Vector3 {0.1, 0.0, 1.4}),
    make_body(2, cube, Vector3 {0.0, 0.0, 0.5}),
  });
  posegen_result_t const boundary = generator.solve(posegen_problem_t {
    .scene = boundary_scene.view,
    .candidate = EntityId {1},
    .targets = {},
    .boundaries = {EntityId {2}},
  });
  // This locks both corrected cone KKT derivatives. The legacy diffsim result
  // is intentionally not the oracle because eval.cpp selected Eigen col(-1)
  // for the cone pose gradient, and divided the cone force gradient by a
  // magnitude built from one tangent and the normal instead of both tangents.
  // Only this case moves: the others settle into near-normal contacts, where
  // the tangential numerator vanishes and both forms agree.
  require_pose_near(
    boundary.optimal_pose,
    Vector3 {
      1.0458150563825335,
      -0.00080235114879420599,
      0.40675094064777667},
    Quaternion {
      0.99976330704095429,
      2.78079000345705e-5,
      7.912251342998839e-5,
      0.021755984484352654},
    2e-6);
  require(std::abs(boundary.c_feq - 0.0062673909671058861) < 1e-9);
  require(std::abs(boundary.c_gap - 0.10325035190866688) < 1e-8);
  require_solver_stats(boundary, config);

  bool rejected_candidate = false;
  try {
    (void)generator.solve(posegen_problem_t {
      .scene = ground_scene.view,
      .candidate = EntityId {2},
      .targets = {},
      .boundaries = {},
    });
  } catch (std::invalid_argument const&) {
    rejected_candidate = true;
  }
  require(rejected_candidate);

  bool rejected_roles = false;
  try {
    (void)generator.solve(posegen_problem_t {
      .scene = target_scene.view,
      .candidate = EntityId {1},
      .targets = {EntityId {3}},
      .boundaries = {EntityId {3}},
    });
  } catch (std::invalid_argument const&) {
    rejected_roles = true;
  }
  require(rejected_roles);

  posegen_config_t invalid = generator.config();
  invalid.force_solver.max_iters = 0;
  bool rejected_config = false;
  try {
    generator.setConfig(invalid);
  } catch (std::invalid_argument const&) {
    rejected_config = true;
  }
  require(rejected_config);
}
