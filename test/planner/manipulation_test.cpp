#include <stacking_core/io/urdf.hpp>
#include <stacking_core/planner.hpp>

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
        "manipulation requirement failed at line " +
        std::to_string(location.line()));
    }
  }

  std::filesystem::path asset(std::string const& name) {
    return std::filesystem::path {__FILE__}.parent_path() / "assets" / name;
  }

  phase_scene_t phase(
    std::shared_ptr<BodyModel const> const& model,
    pose_t const& frame_from_body) {
    std::vector<BodyInstance> bodies;
    bodies.emplace_back(body_instance_config_t {
      .id = EntityId {900},
      .model = model,
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

  void require_continuous(plan_candidate_t const& plan) {
    for (std::size_t i = 1; i < plan.segments.size(); ++i) {
      trajectory_t const& prev = plan.segments[i - 1].trajectory;
      trajectory_t const& next = plan.segments[i].trajectory;
      require(!prev.samples.empty());
      require(!next.samples.empty());
      require(prev.samples.back().robot.positions.isApprox(
        next.samples.front().robot.positions, 1e-12));
    }
  }

}  // namespace

int main() {
  using namespace stacking_core;

  UrdfModel target_urdf = load_urdf_model(asset("stable_pose.urdf"));
  LinkId const target_link = target_urdf.kinematics().findLink("target")->id;
  std::shared_ptr<BodyModel const> const target_model =
    target_urdf.bodyModelPtr(target_link);
  Vector3 const handoff_position {2.0, 3.0, 0.25};
  regrasp_config_t regrasp_config;
  regrasp_config.stable_pose.sampling_level = 3;
  regrasp_config.yaw_samples = 1;
  regrasp_config.max_handoff_orientation_distance = std::numbers::pi;
  regrasp_pose_result_t const poses = generate_regrasp_poses(
    regrasp_pose_problem_t {
      .body_model = target_model,
      .frame_from_body_pick = {},
      .frame_from_body_place = {},
      .handoff_position = handoff_position,
    },
    regrasp_config);
  require(poses.status == solve_status_e::success);
  pose_t const frame_from_target = poses.poses.front().frame_from_body;

  UrdfModel robot_urdf = load_urdf_model(asset("excavator_kinematics.urdf"));
  LinkId const tool = robot_urdf.kinematics().findLink("cs_rotate")->id;
  Eigen::VectorXd q_home(6);
  q_home << 0.1, 0.2, -0.2, -0.1, 0.1, 0.0;
  KinematicState state {FrameId {1}, robot_urdf.kinematicsPtr()};
  state.setPositions(q_home);
  KinematicSnapshot const kinematics = forward_kinematics(state);
  pose_t const frame_from_grasp = kinematics.frameFromLink(tool);
  motion_robot_t robot {
    .initial_state = state,
    .tool_link = tool,
    .link_from_tool = {},
    .gripper_opening = -0.2,
    .ik_initializer = {},
    .collision_bodies = {},
    .self_collision_pairs = {},
  };
  gripper_model_t gripper {
    .state = state,
    .root_link = robot_urdf.kinematics().roots().front(),
    .grasp_from_root = {},
    .opening_offset = Eigen::VectorXd::Zero(6),
    .opening_direction = Eigen::VectorXd::Zero(6),
    .opening_lower = -0.2,
    .opening_upper = 0.0,
    .collision_bodies = {},
    .contact_geometries = {},
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
  joint_grasp_candidate_t const common_grasp {
    .grasp = pick_grasp,
    .positions = {q_home, q_home},
  };
  auto grasp_with_score = [&](Scalar score) {
    joint_grasp_candidate_t candidate = common_grasp;
    candidate.grasp.score = score;
    return candidate;
  };

  direct_plan_problem_t direct_problem {
    .pick = phase(target_model, frame_from_target),
    .place = phase(target_model, frame_from_target),
    .robot = robot,
    .gripper = gripper,
    .grasp_candidates =
      {grasp_with_score(1.0), grasp_with_score(3.0), grasp_with_score(2.0)},
  };
  direct_plan_config_t direct_config;
  direct_config.simulation_refinement = false;
  direct_config.approach_distance = 0.0;
  direct_config.move_steps = 2;
  direct_config.grasp_steps = 2;
  direct_config.motion.max_iters = 1;
  direct_config.max_candidates = 2;
  direct_config.worker_count = 4;
  direct_plan_config_t serial_config = direct_config;
  serial_config.worker_count = 1;
  plan_result_t const serial = solve_direct(direct_problem, serial_config);
  plan_result_t const direct = solve_direct(direct_problem, direct_config);
  require(serial.status == solve_status_e::success);
  require(direct.status == solve_status_e::success);
  require(serial.candidates.size() == direct.candidates.size());
  require(direct.candidates.size() == 2);
  require(direct.candidates[0].score == 3.0);
  require(direct.candidates[1].score == 2.0);
  for (std::size_t i = 0; i < direct.candidates.size(); ++i) {
    require(serial.candidates[i].score == direct.candidates[i].score);
    require(
      serial.candidates[i].segments.size() ==
      direct.candidates[i].segments.size());
    for (std::size_t j = 0; j < direct.candidates[i].segments.size(); ++j) {
      trajectory_t const& lhs = serial.candidates[i].segments[j].trajectory;
      trajectory_t const& rhs = direct.candidates[i].segments[j].trajectory;
      require(lhs.samples.size() == rhs.samples.size());
      for (std::size_t k = 0; k < lhs.samples.size(); ++k) {
        require(lhs.samples[k].robot.positions.isApprox(
          rhs.samples[k].robot.positions, 1e-12));
      }
    }
  }
  require(direct.timings.grasp_candidates == 3);
  require(direct.timings.refined_candidates == 3);
  require(direct.timings.motion_candidates == 3);
  require(direct.timings.total_seconds > 0.0);
  require(direct.selected_candidate() != nullptr);
  require(direct.selected_candidate()->segments.size() == 5);
  require(direct.selected_candidate()->grasp_events.size() == 2);
  require(
    direct.selected_candidate()->segments[2].stage ==
    planning_stage_e::transfer);
  require(
    direct.selected_candidate()->segments[2].mode == motion_mode_e::attached);
  require_continuous(*direct.selected_candidate());
  for (plan_segment_t const& segment : direct.selected_candidate()->segments) {
    for (trajectory_sample_t const& sample : segment.trajectory.samples) {
      require(sample.frame_from_target.has_value());
    }
  }

  attachment_t const attachment {
    .body = direct_problem.pick.target,
    .link = tool,
    .link_from_body =
      compose(inverse(kinematics.frameFromLink(tool)), frame_from_target),
  };
  inhand_plan_config_t inhand_config;
  inhand_config.approach_distance = 0.0;
  inhand_config.move_steps = 2;
  inhand_config.grasp_steps = 2;
  inhand_config.motion.max_iters = 1;
  plan_result_t const inhand = solve_inhand(
    inhand_plan_problem_t {
      .place = direct_problem.place,
      .robot = robot,
      .attachment = attachment,
    },
    inhand_config);
  require(inhand.status == solve_status_e::success);
  require(inhand.selected_candidate() != nullptr);
  require(inhand.selected_candidate()->segments.size() == 3);
  require(inhand.selected_candidate()->grasp_events.size() == 1);
  require(
    inhand.selected_candidate()->grasp_events.front().event ==
    grasp_event_e::release);
  require_continuous(*inhand.selected_candidate());

  direct_plan_problem_t failing_direct = direct_problem;
  failing_direct.grasp_candidates = {joint_grasp_candidate_t {
    .grasp = pick_grasp,
    .positions = {},
  }};
  regrasp_config.approach_distance = 0.0;
  regrasp_config.move_steps = 2;
  regrasp_config.grasp_steps = 2;
  regrasp_config.max_candidates = 1;
  regrasp_config.motion.max_iters = 1;
  pick_place_config_t fallback_config;
  fallback_config.direct = direct_config;
  fallback_config.regrasp = regrasp_config;
  plan_result_t const fallback = solve_pick_place(
    pick_place_problem_t {
      .direct = std::move(failing_direct),
      .handoff = phase(target_model, frame_from_target),
      .handoff_position = handoff_position,
      .pick_grasp_candidates = {pick_grasp},
      .place_grasp_candidates = {place_grasp},
    },
    fallback_config);
  require(fallback.status == solve_status_e::success);
  require(fallback.selected_candidate() != nullptr);
  require(fallback.selected_candidate()->segments.size() == 8);
  require(fallback.selected_candidate()->diagnostics.size() == 1);
  require(
    fallback.selected_candidate()->diagnostics.front().failure.code ==
    "direct_grasp_generation");
}
