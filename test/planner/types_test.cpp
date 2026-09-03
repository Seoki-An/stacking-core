#include <stacking_core/planner.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

  void require(bool condition) {
    if (!condition) {
      throw std::runtime_error("planner types test requirement failed");
    }
  }

}  // namespace

int main() {
  using namespace stacking_core;

  auto body_model = std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {1},
    .inertial = std::nullopt,
    .geometries = {},
  });
  std::vector<BodyInstance> bodies;
  bodies.emplace_back(body_instance_config_t {
    .id = EntityId {1},
    .model = body_model,
    .frame_from_body = pose_t {},
    .motion = motion_t {},
    .mobility = mobility_e::static_body,
  });
  bodies.emplace_back(body_instance_config_t {
    .id = EntityId {2},
    .model = std::move(body_model),
    .frame_from_body = pose_t {},
    .motion = motion_t {},
    .mobility = mobility_e::static_body,
  });
  auto snapshot = std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
    .frame = FrameId {1},
    .bodies = std::move(bodies),
  });
  SceneView pick_view {snapshot, {EntityId {1}}};
  SceneView place_view {snapshot, {EntityId {2}}};
  auto kinematics = std::make_shared<KinematicModel>(kinematic_model_config_t {
    .links = {kinematic_link_t {.id = LinkId {1}, .name = "tool"}},
    .joints = {},
  });
  KinematicState state {FrameId {1}, kinematics};
  direct_plan_problem_t problem {
    .pick = phase_scene_t {std::move(pick_view), EntityId {1}},
    .place = phase_scene_t {std::move(place_view), EntityId {2}},
    .robot =
      motion_robot_t {
        .initial_state = state,
        .tool_link = LinkId {1},
        .link_from_tool = {},
        .gripper_opening = 0.4,
        .ik_initializer = {},
        .collision_bodies = {},
        .self_collision_pairs = {},
      },
    .gripper =
      gripper_model_t {
        .state = state,
        .root_link = LinkId {1},
        .grasp_from_root = {},
        .opening_offset = Eigen::VectorXd {},
        .opening_direction = Eigen::VectorXd {},
        .opening_lower = 0.0,
        .opening_upper = 1.0,
        .collision_bodies = {},
        .contact_geometries = {},
      },
    .grasp_candidates = {},
  };
  require(problem.pick.scene.frame() == FrameId {1});
  require(problem.place.target == EntityId {2});

  trajectory_sample_t sample {
    .robot =
      robot_state_t {
        .positions = problem.robot.initial_state.positions(),
        .gripper_opening = problem.robot.gripper_opening,
      },
    .frame_from_target = pose_t {},
  };
  plan_candidate_t candidate;
  candidate.segments.push_back(plan_segment_t {
    .stage = planning_stage_e::pick_approach,
    .mode = motion_mode_e::free,
    .trajectory = trajectory_t {{sample}},
  });
  candidate.grasp_events.push_back(grasp_event_t {
    .event = grasp_event_e::acquire,
    .segment_index = 0,
    .sample_index = 0,
    .grasp = grasp_t {},
  });
  candidate.score = 3.0;

  plan_result_t plan {
    .status = solve_status_e::success,
    .candidates = {candidate},
    .selected_index = 0,
    .failure = {},
    .timings = {},
  };
  require(plan.selected_candidate() != nullptr);
  require(plan.selected_candidate()->score == 3.0);
  require(
    plan.selected_candidate()->segments.front().trajectory.samples.size() == 1);

  plan.selected_index = 1;
  require(plan.selected_candidate() == nullptr);

  grasp_result_t grasps {
    .status = solve_status_e::success,
    .candidates = {grasp_candidate_t {
      .grasp = {},
      .score = 2.0,
      .contacts = {},
      .solver = {},
      .failure = {},
    }},
    .selected_index = 0,
    .failure = {},
  };
  require(grasps.selected_candidate() != nullptr);
  require(grasps.selected_candidate()->score == 2.0);
}
