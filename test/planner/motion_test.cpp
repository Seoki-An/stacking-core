#include <stacking_core/geometry/dsf_vert.hpp>
#include <stacking_core/io/urdf.hpp>
#include <stacking_core/planner.hpp>

#include <filesystem>
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
        "motion-planning requirement failed at line " +
        std::to_string(location.line()));
    }
  }

  std::filesystem::path asset(std::string const& name) {
    return std::filesystem::path {__FILE__}.parent_path() / "assets" / name;
  }

  std::shared_ptr<BodyModel const> make_cube_model() {
    Matrix3X nodes(3, 8);
    nodes << -0.1, -0.1, -0.1, -0.1, 0.1, 0.1, 0.1, 0.1, -0.1, -0.1, 0.1, 0.1,
      -0.1, -0.1, 0.1, 0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1;
    std::vector<geometry_config_t> geometries;
    geometries.push_back(dsf_vert_geometry_config_t {
      .properties =
        geometry_properties_t {
          .id = GeometryId {1000},
          .body_from_geometry = pose_t {},
          .material = material_t {},
        },
      .nodes = std::move(nodes),
      .sharpness = 20,
    });
    return std::make_shared<BodyModel>(body_model_config_t {
      .id = BodyModelId {1000},
      .inertial = std::nullopt,
      .geometries = std::move(geometries),
    });
  }

}  // namespace

int main() {
  using namespace stacking_core;

  UrdfModel const urdf = load_urdf_model(asset("excavator_kinematics.urdf"));
  LinkId const tool = urdf.kinematics().findLink("cs_rotate")->id;
  Eigen::VectorXd positions(6);
  positions << 0.1, 0.2, -0.2, -0.1, 0.1, 0.0;
  KinematicState initial_state {FrameId {1}, urdf.kinematicsPtr()};
  initial_state.setPositions(positions);

  std::shared_ptr<BodyModel const> const cube = make_cube_model();
  pose_t const frame_from_tool =
    forward_kinematics(initial_state).frameFromLink(tool);
  std::vector<BodyInstance> obstacles;
  obstacles.emplace_back(body_instance_config_t {
    .id = EntityId {1000},
    .model = cube,
    .frame_from_body = frame_from_tool,
    .motion = {},
    .mobility = mobility_e::static_body,
  });
  auto snapshot = std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
    .frame = FrameId {1},
    .bodies = std::move(obstacles),
  });
  SceneView const scene {snapshot, {EntityId {1000}}};

  auto make_robot = [&] {
    return motion_robot_t {
      .initial_state = initial_state,
      .tool_link = tool,
      .link_from_tool = pose_t {},
      .gripper_opening = 0.0,
      .ik_initializer = {},
      .collision_bodies = {motion_link_body_t {
        .link = tool,
        .entity = EntityId {1001},
        .body_model = cube,
      }},
      .self_collision_pairs = {},
    };
  };
  motion_waypoint_t const stationary {
    .goal = joint_goal_t {.positions = positions, .preserve_branch = false},
    .steps = 2,
  };

  motion_result_t const colliding = solve_free_motion(
    free_motion_problem_t {
      .scene = scene,
      .robot = make_robot(),
      .waypoints = {stationary},
    },
    motion_planning_config_t {.max_iters = 1});
  require(colliding.status == solve_status_e::infeasible);
  require(colliding.failure.code == "collision_or_joint_limit");
  require(colliding.failure.message.find("collision entities=") != std::string::npos);
  require(colliding.failure.message.find("gap_m=") != std::string::npos);
  require(colliding.failure.message.find("sample=") != std::string::npos);
  motion_planning_config_t alm_config;
  alm_config.collision_alm_enabled = true;
  alm_config.collision_alm_max_iters = 2;
  alm_config.max_iters = 1;
  auto const still_colliding = solve_free_motion(
    free_motion_problem_t {scene, make_robot(), {stationary}}, alm_config);
  require(still_colliding.status == solve_status_e::infeasible);
  require(still_colliding.failure.code == "collision_or_joint_limit");
  require(still_colliding.failure.message.find("collision entities=") != std::string::npos);
  alm_config.collision_alm_beta_increase = 0.0;
  require(solve_free_motion(
    free_motion_problem_t {scene, make_robot(), {stationary}}, alm_config).status ==
    solve_status_e::invalid_problem);

  motion_robot_t invalid_robot = make_robot();
  invalid_robot.self_collision_pairs.push_back(collision_body_pair_t {
    .first = EntityId {1001}, .second = EntityId {1001}});
  motion_result_t const invalid_pair = solve_free_motion(free_motion_problem_t {
    .scene = scene,
    .robot = std::move(invalid_robot),
    .waypoints = {stationary},
  });
  require(invalid_pair.status == solve_status_e::invalid_problem);
  require(invalid_pair.failure.code == "invalid_self_collision_pair");

  motion_result_t const missing_waypoints =
    solve_free_motion(free_motion_problem_t {
      .scene = scene,
      .robot = make_robot(),
      .waypoints = {},
    });
  require(missing_waypoints.status == solve_status_e::invalid_problem);
  require(missing_waypoints.failure.code == "missing_waypoints");
}
