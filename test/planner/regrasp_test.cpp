#include <stacking_core/io/urdf.hpp>
#include <stacking_core/planner.hpp>

#include <cmath>
#include <filesystem>
#include <memory>
#include <numbers>
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
        "regrasp requirement failed at line " +
        std::to_string(location.line()));
    }
  }

  std::filesystem::path asset(std::string const& name) {
    return std::filesystem::path {__FILE__}.parent_path() / "assets" / name;
  }

  phase_scene_t make_phase(
    std::shared_ptr<BodyModel const> const& body_model,
    pose_t const& frame_from_body) {
    std::vector<BodyInstance> bodies;
    bodies.emplace_back(body_instance_config_t {
      .id = EntityId {900},
      .model = body_model,
      .frame_from_body = frame_from_body,
      .motion = {},
      .mobility = mobility_e::kinematic,
    });
    auto snapshot = std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
      .frame = FrameId {1},
      .bodies = std::move(bodies),
    });
    return phase_scene_t {
      .scene = SceneView {std::move(snapshot), {EntityId {900}}},
      .target = EntityId {900},
    };
  }

}  // namespace

int main() {
  using namespace stacking_core;

  UrdfModel target_urdf = load_urdf_model(asset("stable_pose.urdf"));
  LinkId const target_link = target_urdf.kinematics().findLink("target")->id;
  std::shared_ptr<BodyModel const> const target_model =
    target_urdf.bodyModelPtr(target_link);
  regrasp_config_t pose_config;
  pose_config.stable_pose.sampling_level = 3;
  pose_config.yaw_samples = 1;
  pose_config.max_handoff_orientation_distance = std::numbers::pi;
  Vector3 const handoff_position {2.0, 3.0, 0.25};
  regrasp_pose_result_t const poses = generate_regrasp_poses(
    regrasp_pose_problem_t {
      .body_model = target_model,
      .frame_from_body_pick = pose_t {},
      .frame_from_body_place = pose_t {},
      .handoff_position = handoff_position,
    },
    pose_config);
  require(poses.status == solve_status_e::success);
  require(poses.poses.size() == 8);
  // Legacy regrasp_pose_at_yaw result for the copied model_2 fixture, first
  // stable direction, yaw=0, and handoff plane z=0.25.
  pose_t const legacy_first_pose {
    Vector3 {1.9360389697102838, 3.0487412120306021, 0.80370894430997319},
    Quaternion {
      -0.1110344696677312, 0.78440235419730431, -0.60420965929140547,
      -0.08552766157387999},
  };
  require(poses.poses.front().frame_from_body.position.isApprox(
    legacy_first_pose.position, 1e-8));
  require(
    poses.poses.front().frame_from_body.orientation.angularDistance(
      legacy_first_pose.orientation) < 1e-8);
  for (regrasp_pose_t const& candidate : poses.poses) {
    require(
      (transform_vector(
         candidate.frame_from_body, candidate.stable_pose.resting_dir_body) +
       Vector3::UnitZ())
        .norm() < 2e-12);
    Vector3 const contact = transform_point(
      candidate.frame_from_body, candidate.stable_pose.support_point_body);
    require(std::abs(contact.z() - handoff_position.z()) < 2e-12);
  }

  UrdfModel robot_urdf = load_urdf_model(asset("excavator_kinematics.urdf"));
  LinkId const tool = robot_urdf.kinematics().findLink("cs_rotate")->id;
  Eigen::VectorXd q_home(6);
  q_home << 0.1, 0.2, -0.2, -0.1, 0.1, 0.0;
  KinematicState state {FrameId {1}, robot_urdf.kinematicsPtr()};
  state.setPositions(q_home);
  pose_t const frame_from_grasp = forward_kinematics(state).frameFromLink(tool);
  pose_t const frame_from_target = poses.poses.front().frame_from_body;

  motion_robot_t robot {
    .initial_state = state,
    .tool_link = tool,
    .link_from_tool = pose_t {},
    .gripper_opening = -0.2,
    .ik_initializer = {},
    .collision_bodies = {},
    .self_collision_pairs = {},
  };
  grasp_candidate_t const pick_grasp {
    .grasp = grasp_t {.frame_from_grasp = frame_from_grasp, .opening = -0.1},
    .score = 3.0,
    .contacts = {},
    .solver = {},
    .failure = {},
  };
  grasp_candidate_t const place_grasp {
    .grasp = grasp_t {.frame_from_grasp = frame_from_grasp, .opening = -0.1},
    .score = 5.0,
    .contacts = {},
    .solver = {},
    .failure = {},
  };
  regrasp_config_t config = pose_config;
  config.approach_distance = 0.0;
  config.move_steps = 2;
  config.grasp_steps = 2;
  config.max_candidates = 1;
  config.motion.max_iters = 1;
  plan_result_t const plan = solve_regrasp(
    regrasp_problem_t {
      .pick = make_phase(target_model, frame_from_target),
      .handoff = make_phase(target_model, frame_from_target),
      .place = make_phase(target_model, frame_from_target),
      .robot = std::move(robot),
      .pick_grasps = {pick_grasp},
      .place_grasps = {place_grasp},
      .handoff_position = handoff_position,
    },
    config);
  require(plan.status == solve_status_e::success);
  require(plan.selected_candidate() != nullptr);
  require(plan.selected_candidate()->segments.size() == 8);
  require(plan.selected_candidate()->grasp_events.size() == 4);
  require(std::abs(plan.selected_candidate()->score - 8.0) < 1e-12);
  require(
    plan.selected_candidate()->segments[0].stage ==
    planning_stage_e::pick_approach);
  require(
    plan.selected_candidate()->segments[7].stage ==
    planning_stage_e::place_retreat);
  for (plan_segment_t const& segment : plan.selected_candidate()->segments) {
    require(!segment.trajectory.samples.empty());
    for (trajectory_sample_t const& sample : segment.trajectory.samples) {
      require(sample.robot.positions.allFinite());
      require(sample.frame_from_target.has_value());
    }
  }
  require(
    plan.selected_candidate()->grasp_events[0].event == grasp_event_e::acquire);
  require(
    plan.selected_candidate()->grasp_events[1].event == grasp_event_e::release);
  require(
    plan.selected_candidate()->grasp_events[2].event == grasp_event_e::acquire);
  require(
    plan.selected_candidate()->grasp_events[3].event == grasp_event_e::release);

  regrasp_problem_t invalid_problem {
    .pick = make_phase(target_model, frame_from_target),
    .handoff = make_phase(target_model, frame_from_target),
    .place = make_phase(target_model, frame_from_target),
    .robot =
      motion_robot_t {
        .initial_state = state,
        .tool_link = tool,
        .link_from_tool = pose_t {},
        .gripper_opening = 0.0,
        .ik_initializer = {},
        .collision_bodies = {},
        .self_collision_pairs = {},
      },
    .pick_grasps = {},
    .place_grasps = {place_grasp},
    .handoff_position = handoff_position,
  };
  plan_result_t const invalid = solve_regrasp(invalid_problem, config);
  require(invalid.status == solve_status_e::invalid_problem);
  require(invalid.failure.code == "missing_grasps");

  regrasp_config_t invalid_motion_config = config;
  invalid_motion_config.motion.max_iters = 0;
  invalid_problem.pick_grasps = {pick_grasp};
  plan_result_t const invalid_motion =
    solve_regrasp(invalid_problem, invalid_motion_config);
  require(invalid_motion.status == solve_status_e::invalid_problem);
  require(invalid_motion.failure.code == "regrasp_pick_approach_motion");
}
