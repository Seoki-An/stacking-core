#include <stacking_core/io/urdf.hpp>
#include <stacking_core/planner.hpp>

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

  using namespace stacking_core;

  using joint_sample_t = std::array<Scalar, 6>;
  using joint_path_t = std::array<joint_sample_t, 8>;

  void require(
    bool condition,
    std::source_location location = std::source_location::current()) {
    if (!condition) {
      throw std::runtime_error(
        "planner motion-reference requirement failed at line " +
        std::to_string(location.line()));
    }
  }

  joint_path_t legacy_free_path() {
    return {{
      {0.1, 0.2, -0.2, -0.1, 0.1, 0.0},
      {0.12856246198253443, 0.21428123099126722, -0.21428123099126722,
       -0.11428123099126722, 0.11428123099126722, 0.02856246198253445},
      {0.1335035211977611, 0.21675176059888057, -0.21675176059888057,
       -0.11675176059888054, 0.11675176059888054, 0.033503521197761107},
      {0.14225312388214761, 0.2211265619410738, -0.2211265619410738,
       -0.1211265619410738, 0.1211265619410738, 0.042253123882147636},
      {0.15774687611785237, 0.22887343805892621, -0.22887343805892621,
       -0.1288734380589262, 0.1288734380589262, 0.057746876117852397},
      {0.16649647880223892, 0.23324823940111944, -0.23324823940111944,
       -0.13324823940111943, 0.13324823940111943, 0.066496478802238926},
      {0.17143753801746558, 0.23571876900873279, -0.23571876900873279,
       -0.13571876900873278, 0.13571876900873278, 0.071437538017465579},
      {0.2, 0.25, -0.25, -0.15, 0.15, 0.1},
    }};
  }

  joint_path_t legacy_carry_path() {
    return {{
      {0.1, 0.2, -0.2, -0.1, 0.1, 0.0},
      {0.13847715913032224, 0.21666499566537578, -0.21338034902806427,
       -0.11452996188682894, 0.11375854083668245, 0.028391244833816005},
      {0.1451333983003604, 0.21954791210344063, -0.21569505885239901,
       -0.11704352250822374, 0.11613866301046424, 0.033302712981187921},
      {0.15692021161621095, 0.2246529473769249, -0.2197939011784231,
       -0.12149452965841805, 0.1203533605416818, 0.041999893143249416},
      {0.17779221968232359, 0.23369290251902378, -0.22705208538121055,
       -0.12937635097601602, 0.12781671499140879, 0.057400793308077479},
      {0.18957903306468629, 0.23879794046340644, -0.23115092954899236,
       -0.13382738775993805, 0.13203143098921197, 0.066098013151766541},
      {0.19623527236880448, 0.24168084131736289, -0.23346559608241813,
       -0.13634096494361639, 0.13441154978673189, 0.071009466014879952},
      {0.19999418351130224, 0.24330886983230826, -0.23477272330466215,
       -0.13776045024085251, 0.1357556493206773, 0.073783064636383427},
    }};
  }

  std::array<pose_t, 8> legacy_carry_target_path() {
    return {{
      pose_t {
        Vector3 {8.8387850597223458, 1.0566097724763823, 3.3000641283748138},
        Quaternion {
          -0.0010464195147442659, -0.76580895569574681, 0.00079574398633250115,
          0.64306680459645704},
      },
      pose_t {
        Vector3 {8.778623812986714, 1.3949460071272795, 3.3935834382727221},
        Quaternion {
          -0.020832896816947118, -0.76218275862361384, 0.00067220310910901383,
          0.64702625990765794},
      },
      pose_t {
        Vector3 {8.7667244168652871, 1.453132932891668, 3.4097257889292907},
        Quaternion {
          -0.024287190055580669, -0.76154831130081768, 0.00065724373074415796,
          0.64765258432647022},
      },
      pose_t {
        Vector3 {8.7445802896177991, 1.5558969186074596, 3.4382840538508077},
        Quaternion {
          -0.030426065359511652, -0.76041966713716547, 0.00063544002752981103,
          0.64871856809691597},
      },
      pose_t {
        Vector3 {8.7020198332782996, 1.7369608929324682, 3.4887704830359048},
        Quaternion {
          -0.041364371727172937, -0.75840485553975767, 0.00061165203640743725,
          0.65046959170058216},
      },
      pose_t {
        Vector3 {8.6761054682715262, 1.8386631051913445, 3.5172326370022122},
        Quaternion {
          -0.047578674145897171, -0.75725788483169643, 0.00060669047453841551,
          0.65138037854498809},
      },
      pose_t {
        Vector3 {8.6608749171617916, 1.8959119329557244, 3.5332900637392957},
        Quaternion {
          -0.051099591009682206, -0.7566072563424725, 0.00060658927863612962,
          0.65186956018670372},
      },
      pose_t {
        Vector3 {8.6520843951886626, 1.9281809751031367, 3.5423529674264849},
        Quaternion {
          -0.053091571550721979, -0.75623889879198092, 0.00060740353603930155,
          0.65213774928699741},
      },
    }};
  }

  robot_state_t make_robot_state(joint_sample_t const& sample) {
    robot_state_t state;
    state.positions.resize(static_cast<Eigen::Index>(sample.size()));
    for (std::size_t i = 0; i < sample.size(); ++i) {
      state.positions(static_cast<Eigen::Index>(i)) = sample[i];
    }
    state.gripper_opening = -0.2;
    return state;
  }

  std::filesystem::path asset(std::string const& name) {
    return std::filesystem::path {__FILE__}.parent_path() / "assets" / name;
  }

}  // namespace

int main() {
  using namespace stacking_core;

  // Emitted by diffsim commit 0c433b2. Both solves use the legacy excavator,
  // q_init=[0.1,0.2,-0.2,-0.1,0.1,0], q_goal=[0.2,0.25,-0.25,-0.15,0.15,0.1],
  // eight samples, and an obstacle translated to [100,100,100]. The carried
  // solve uses end_from_target translation [0.1,-0.2,0.3] and a 0.2-radian
  // rotation about +Y. These constants become direct parity expectations once
  // the motion solvers are ported.
  UrdfModel urdf = load_urdf_model(asset("excavator_kinematics.urdf"));
  LinkId const end = urdf.kinematics().findLink("cs_rotate")->id;
  Eigen::VectorXd q_init(6), q_goal(6);
  q_init << 0.1, 0.2, -0.2, -0.1, 0.1, 0.0;
  q_goal << 0.2, 0.25, -0.25, -0.15, 0.15, 0.1;

  auto target_model = std::make_shared<BodyModel>(body_model_config_t {
    .id = BodyModelId {900},
    .inertial = std::nullopt,
    .geometries = {},
  });
  std::vector<BodyInstance> bodies;
  bodies.emplace_back(body_instance_config_t {
    .id = EntityId {900},
    .model = target_model,
    .frame_from_body = pose_t {},
    .motion = {},
    .mobility = mobility_e::kinematic,
  });
  auto snapshot = std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
    .frame = FrameId {1},
    .bodies = std::move(bodies),
  });
  SceneView scene {snapshot, {EntityId {900}}};
  KinematicState initial_state {FrameId {1}, urdf.kinematicsPtr()};
  initial_state.setPositions(q_init);

  motion_robot_t free_robot {
    .initial_state = initial_state,
    .tool_link = end,
    .link_from_tool = pose_t {},
    .gripper_opening = -0.2,
    .ik_initializer = {},
    .collision_bodies = {},
    .self_collision_pairs = {},
  };
  motion_result_t const free = solve_free_motion(free_motion_problem_t {
    .scene = scene,
    .robot = std::move(free_robot),
    .waypoints = {motion_waypoint_t {
      .goal = joint_goal_t {.positions = q_goal, .preserve_branch = false},
      .steps = 8,
    }},
  });

  motion_robot_t grasped_robot {
    .initial_state = initial_state,
    .tool_link = end,
    .link_from_tool = pose_t {},
    .gripper_opening = -0.2,
    .ik_initializer = {},
    .collision_bodies = {},
    .self_collision_pairs = {},
  };
  pose_t const end_from_target {
    Vector3 {0.1, -0.2, 0.3},
    Quaternion {Eigen::AngleAxis<Scalar> {0.2, Vector3::UnitY()}},
  };
  motion_result_t const carry = solve_grasped_motion(grasped_motion_problem_t {
    .scene = scene,
    .robot = std::move(grasped_robot),
    .attachment =
      attachment_t {
        .body = EntityId {900},
        .link = end,
        .link_from_body = end_from_target,
      },
    .waypoints = {motion_waypoint_t {
      .goal = joint_goal_t {.positions = q_goal, .preserve_branch = false},
      .steps = 8,
    }},
  });

  require(free.status == solve_status_e::success);
  require(carry.status == solve_status_e::success);
  require(free.trajectory.samples.size() == 8);
  require(carry.trajectory.samples.size() == 8);
  joint_path_t const expected_free = legacy_free_path();
  joint_path_t const expected_carry = legacy_carry_path();
  std::array<pose_t, 8> const expected_targets = legacy_carry_target_path();
  for (std::size_t i = 0; i < expected_free.size(); ++i) {
    require(carry.trajectory.samples[i].frame_from_target.has_value());
    require(
      (free.trajectory.samples[i].robot.positions -
       make_robot_state(expected_free[i]).positions)
        .norm() < 2e-7);
    require(
      (carry.trajectory.samples[i].robot.positions -
       make_robot_state(expected_carry[i]).positions)
        .norm() < 2e-11);
    require(
      (carry.trajectory.samples[i].frame_from_target->position -
       expected_targets[i].position)
        .norm() < 2e-12);
    require(
      carry.trajectory.samples[i]
        .frame_from_target->orientation.angularDistance(
          expected_targets[i].orientation) < 2e-12);
  }
  require(!free.trajectory.samples.front().frame_from_target.has_value());

  for (trajectory_sample_t const& sample : carry.trajectory.samples) {
    require(sample.robot.positions.allFinite());
    require(sample.frame_from_target.has_value());
    require(sample.frame_from_target->position.allFinite());
    require(
      std::abs(sample.frame_from_target->orientation.norm() - 1.0) < 2e-12);
  }

  // The legacy carried solver intentionally relaxes the terminal joint goal;
  // preserving this distinction prevents a future port from silently being
  // compared against the exact endpoint behavior of free motion.
  require(!carry.trajectory.samples.back().robot.positions.isApprox(
    make_robot_state(legacy_free_path().back()).positions, 1e-4));

  // The same deterministic run returns an infeasible single-grasp candidate.
  // Retain the numerical iterate so infeasibility behavior can also be checked.
  grasp_result_t const grasp {
    .status = solve_status_e::infeasible,
    .candidates = {grasp_candidate_t {
      .grasp =
        grasp_t {
          .frame_from_grasp =
            pose_t {
              Vector3 {
                0.07205321832890535, -0.36470639117367643, 2.0164649650782032},
              Quaternion {
                0.79444642638240659, -0.60453151665095906, 0.055494432052871363,
                -0.017801376212079889},
            },
          .opening = -0.28144756195666654,
        },
      .score = -1.7537283242726778,
      .contacts = {},
      .solver = {},
      .failure =
        {
          .code = "legacy_infeasible",
          .message = "legacy single-grasp solve did not satisfy feasibility",
          .retryable = true,
        },
    }},
    .selected_index = std::nullopt,
    .failure =
      {
        .code = "legacy_infeasible",
        .message = "legacy single-grasp solve did not satisfy feasibility",
        .retryable = true,
      },
  };
  require(grasp.selected_candidate() == nullptr);
  require(grasp.candidates.size() == 1);
  require(std::isfinite(grasp.candidates.front().score));
}
