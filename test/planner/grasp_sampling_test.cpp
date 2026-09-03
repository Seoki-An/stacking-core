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
        "grasp sampling test requirement failed at line " +
        std::to_string(location.line()));
    }
  }

  std::filesystem::path asset(std::string const& name) {
    return std::filesystem::path {__FILE__}.parent_path() / "assets" / name;
  }

  Matrix3X cube_nodes(Scalar half_extent) {
    Matrix3X nodes(3, 8);
    nodes << -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0,
      -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0;
    return half_extent * nodes;
  }

  std::shared_ptr<BodyModel const> cube_model(Scalar half_extent) {
    std::vector<geometry_config_t> geometries;
    geometries.emplace_back(dsf_vert_geometry_config_t {
      .properties =
        geometry_properties_t {
          .id = GeometryId {1},
          .body_from_geometry = {},
          .material = {},
        },
      .nodes = cube_nodes(half_extent),
      .sharpness = 20,
    });
    return std::make_shared<BodyModel>(body_model_config_t {
      .id = BodyModelId {1},
      .inertial = std::nullopt,
      .geometries = std::move(geometries),
    });
  }

  std::shared_ptr<BodyModel const> plane_model() {
    std::vector<geometry_config_t> geometries;
    geometries.emplace_back(plane_geometry_config_t {
      .properties =
        geometry_properties_t {
          .id = GeometryId {2},
          .body_from_geometry = {},
          .material = {},
        },
    });
    return std::make_shared<BodyModel>(body_model_config_t {
      .id = BodyModelId {2},
      .inertial = std::nullopt,
      .geometries = std::move(geometries),
    });
  }

  gripper_model_t gripper() {
    UrdfModel model = load_urdf_model(asset("grasp_reference.urdf"));
    KinematicState state {FrameId {1}, model.kinematicsPtr()};
    LinkId const root = model.kinematics().findLink("root")->id;
    LinkId const left = model.kinematics().findLink("left_pad")->id;
    LinkId const right = model.kinematics().findLink("right_pad")->id;
    GeometryId const left_geometry = model.bodyModel(left).geometry(0).id();
    GeometryId const right_geometry = model.bodyModel(right).geometry(0).id();
    return gripper_model_t {
      .state = std::move(state),
      .root_link = root,
      .grasp_from_root = {},
      .opening_offset = Eigen::VectorXd::Zero(2),
      .opening_direction = Eigen::VectorXd::Ones(2),
      .opening_lower = -0.1,
      .opening_upper = 0.1,
      .collision_bodies =
        {
          grasp_link_body_t {left, EntityId {101}, model.bodyModelPtr(left)},
          grasp_link_body_t {right, EntityId {102}, model.bodyModelPtr(right)},
        },
      .contact_geometries =
        {
          grasp_contact_geometry_t {
            EntityId {101}, left_geometry, grasp_contact_side_e::left},
          grasp_contact_geometry_t {
            EntityId {102}, right_geometry, grasp_contact_side_e::right},
        },
    };
  }

  phase_scene_t phase(bool with_blocking_plane) {
    std::vector<BodyInstance> bodies;
    auto target = cube_model(1.0);
    bodies.emplace_back(body_instance_config_t {
      .id = EntityId {1},
      .model = std::move(target),
      .frame_from_body = {},
      .motion = {},
      .mobility = mobility_e::static_body,
    });
    std::vector<EntityId> ids {EntityId {1}};
    if (with_blocking_plane) {
      bodies.emplace_back(body_instance_config_t {
        .id = EntityId {2},
        .model = plane_model(),
        .frame_from_body =
          pose_t {Vector3 {0.0, 0.0, 10.0}, Quaternion::Identity()},
        .motion = {},
        .mobility = mobility_e::static_body,
      });
      ids.push_back(EntityId {2});
    }
    auto snapshot = std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
      .frame = FrameId {1},
      .bodies = std::move(bodies),
    });
    return phase_scene_t {
      .scene = SceneView {std::move(snapshot), std::move(ids)},
      .target = EntityId {1},
    };
  }

  KinematicState cartesian_state(pose_t const& frame_from_root) {
    std::vector<kinematic_link_t> links;
    for (std::uint64_t id = 10; id <= 16; ++id) {
      links.push_back(kinematic_link_t {
        .id = LinkId {id},
        .name = "link_" + std::to_string(id),
      });
    }
    auto joint = [](
                   std::uint64_t id, joint_type_e type, LinkId parent,
                   LinkId child, Vector3 axis, Scalar lower, Scalar upper) {
      return kinematic_joint_t {
        .id = JointId {id},
        .name = "joint_" + std::to_string(id),
        .type = type,
        .parent = parent,
        .child = child,
        .parent_from_child_zero = {},
        .axis = axis,
        .limit = joint_limit_t {lower, upper},
        .mimic = std::nullopt,
      };
    };
    Scalar const pi = std::numbers::pi_v<Scalar>;
    auto model = std::make_shared<KinematicModel>(kinematic_model_config_t {
      .links = std::move(links),
      .joints =
        {
          joint(
            10, joint_type_e::prismatic, LinkId {10}, LinkId {11},
            Vector3::UnitX(), -5.0, 5.0),
          joint(
            11, joint_type_e::prismatic, LinkId {11}, LinkId {12},
            Vector3::UnitY(), -5.0, 5.0),
          joint(
            12, joint_type_e::prismatic, LinkId {12}, LinkId {13},
            Vector3::UnitZ(), -5.0, 5.0),
          joint(
            13, joint_type_e::revolute, LinkId {13}, LinkId {14},
            Vector3::UnitX(), -pi, pi),
          joint(
            14, joint_type_e::revolute, LinkId {14}, LinkId {15},
            Vector3::UnitY(), -pi, pi),
          joint(
            15, joint_type_e::revolute, LinkId {15}, LinkId {16},
            Vector3::UnitZ(), -pi, pi),
        },
    });
    KinematicState state {FrameId {1}, std::move(model)};
    state.setFrameFromRoot(LinkId {10}, frame_from_root);
    return state;
  }

}  // namespace

int main() {
  using namespace stacking_core;

  grasp_sampling_problem_t problem {
    .phases = {phase(false)},
    .gripper = gripper(),
  };
  grasp_sampling_config_t surface_config;
  surface_config.max_seeds = 4;
  surface_config.dir_samples = 4;
  surface_config.spin_samples = 1;
  surface_config.include_flipped = false;
  surface_config.retreat_distance = 0.2;
  grasp_seed_result_t const surface =
    generate_grasp_seeds(problem, surface_config);
  require(surface.status == solve_status_e::success);
  require(surface.seeds.size() == 4);
  require(surface.rejected_width == 0);
  // Emitted by legacy diffsim's initialize_surface_grasp() for the same DSF,
  // first four-direction Fibonacci sample, and 0.2 m retreat.
  grasp_seed_t const& reference = surface.seeds.front();
  require(reference.contact_positive.isApprox(
    Vector3 {1.0374608623657866, 0.0, 1.0207432278671402}, 1e-12));
  require(reference.contact_negative.isApprox(
    Vector3 {-1.0374608623657866, -3.748935701362472e-17, -1.0207432278671402},
    1e-12));
  require(reference.grasp.frame_from_grasp.position.isApprox(
    Vector3 {
      -0.14026804313647648, -2.1279019288957365e-17, 0.14256533966803983},
    1e-12));
  Matrix3 expected_R;
  expected_R << 0.71282669834019918, 0.70134021568238236,
    -5.140629274320585e-18, -5.140629274320585e-18, 0.0, 1.0,
    0.70134021568238236, -0.71282669834019929, 0.0;
  require(
    reference.grasp.frame_from_grasp.orientation.toRotationMatrix().isApprox(
      expected_R, 1e-12));
  for (grasp_seed_t const& seed : surface.seeds) {
    Matrix3 const R =
      seed.grasp.frame_from_grasp.orientation.toRotationMatrix();
    require(R.transpose().isApprox(R.inverse(), 1e-12));
    require(std::abs(R.determinant() - 1.0) < 1e-12);
    Vector3 const midpoint =
      0.5 * (seed.contact_positive + seed.contact_negative);
    require(
      std::abs(
        (midpoint - seed.grasp.frame_from_grasp.position).dot(R.col(1)) -
        surface_config.retreat_distance) < 1e-12);
    require(std::abs(seed.grasp.opening - (-0.05)) < 1e-12);
  }

  grasp_sampling_config_t width_config = surface_config;
  width_config.max_target_width = 0.1;
  grasp_seed_result_t const too_wide =
    generate_grasp_seeds(problem, width_config);
  require(too_wide.status == solve_status_e::infeasible);
  require(too_wide.seeds.empty());
  require(too_wide.rejected_width == 4);

  grasp_sampling_config_t parallel_config;
  parallel_config.strategy = grasp_sampling_strategy_e::parallel_jaw;
  parallel_config.max_seeds = 1;
  parallel_config.spin_samples = 1;
  parallel_config.include_flipped = false;
  grasp_seed_result_t const parallel =
    generate_grasp_seeds(problem, parallel_config);
  require(parallel.status == solve_status_e::success);
  require(parallel.seeds.size() == 1);
  grasp_seed_t const& parallel_seed = parallel.seeds.front();
  Matrix3 const parallel_R =
    parallel_seed.grasp.frame_from_grasp.orientation.toRotationMatrix();
  require(parallel_R.col(1).isApprox(-Vector3::UnitZ(), 1e-12));
  require(parallel_seed.grasp.opening >= problem.gripper.opening_lower);
  require(parallel_seed.grasp.opening <= problem.gripper.opening_upper);

  grasp_sampling_problem_t blocked_problem {
    .phases = {phase(true)},
    .gripper = gripper(),
  };
  grasp_sampling_config_t clearance_config = surface_config;
  clearance_config.scene_clearance_margin = 1e-3;
  grasp_seed_result_t const blocked =
    generate_grasp_seeds(blocked_problem, clearance_config);
  require(blocked.status == solve_status_e::infeasible);
  require(blocked.seeds.empty());
  require(blocked.rejected_clearance == 4);

  grasp_generation_config_t generation_config;
  generation_config.trust_region.max_iters = 20;
  generation_config.alm.max_iters = 10;
  generation_config.alm.inequality_tol = 1e-2;
  grasp_result_t const sampled =
    sample_grasps(problem, parallel_config, generation_config);
  if (sampled.status != solve_status_e::success) {
    std::string detail = sampled.failure.code + ": " + sampled.failure.message;
    if (!sampled.candidates.empty()) {
      detail += " / " + sampled.candidates.front().failure.code + ": " +
        sampled.candidates.front().failure.message;
    }
    throw std::runtime_error(detail);
  }
  require(sampled.selected_candidate() != nullptr);
  require(sampled.selected_candidate()->solver.converged);

  grasp_sampling_config_t serial_sampling_config = parallel_config;
  serial_sampling_config.max_seeds = 15;
  serial_sampling_config.worker_count = 1;
  grasp_result_t const serial_sampled =
    sample_grasps(problem, serial_sampling_config, generation_config);
  grasp_sampling_config_t concurrent_sampling_config = serial_sampling_config;
  concurrent_sampling_config.worker_count = 4;
  grasp_result_t const concurrent_sampled =
    sample_grasps(problem, concurrent_sampling_config, generation_config);
  require(concurrent_sampled.status == serial_sampled.status);
  require(
    concurrent_sampled.candidates.size() == serial_sampled.candidates.size());
  for (std::size_t i = 0; i < serial_sampled.candidates.size(); ++i) {
    grasp_candidate_t const& serial_candidate = serial_sampled.candidates[i];
    grasp_candidate_t const& concurrent_candidate =
      concurrent_sampled.candidates[i];
    require(
      std::abs(concurrent_candidate.score - serial_candidate.score) < 1e-12);
    require(concurrent_candidate.grasp.frame_from_grasp.position.isApprox(
      serial_candidate.grasp.frame_from_grasp.position, 1e-12));
    require(
      concurrent_candidate.grasp.frame_from_grasp.orientation.angularDistance(
        serial_candidate.grasp.frame_from_grasp.orientation) < 1e-12);
    require(concurrent_candidate.failure.code == serial_candidate.failure.code);
  }

  grasp_simulation_config_t simulation_config;
  simulation_config.steps = 400;
  grasp_simulation_result_t const simulated = simulate_grasp(
    grasp_simulation_problem_t {
      .phase = problem.phases.front(),
      .gripper = problem.gripper,
      .initial_grasp = sampled.selected_candidate()->grasp,
    },
    simulation_config);
  if (simulated.status != solve_status_e::success) {
    throw std::runtime_error(
      simulated.failure.code +
      ": v=" + std::to_string(simulated.terminal_velocity_norm) +
      ", left=" + std::to_string(simulated.left_contact) +
      ", right=" + std::to_string(simulated.right_contact));
  }
  require(simulated.trajectory.size() == 400);
  require(simulated.left_contact);
  require(simulated.right_contact);
  require(simulated.grasp.frame_from_grasp.position.allFinite());
  require(std::isfinite(simulated.grasp.opening));
  // Quasi-dynamic update preserved from legacy diffsim: identity generalized
  // inertia, 80 projected Gauss-Seidel iterations, and the stored pre-step
  // terminal grasp convention.
  require(simulated.grasp.frame_from_grasp.position.isApprox(
    Vector3 {0.000002, 0.0, -0.373088}, 1e-5));
  require(std::abs(simulated.grasp.opening - 0.075787) < 1e-5);
  require(
    simulated.terminal_velocity_norm < simulation_config.settled_velocity_tol);

  bool initializer_called = false;
  joint_grasp_result_t const sampled_joint = sample_joint_grasps(
    joint_grasp_sampling_problem_t {
      .grasp = problem,
      .initial_states = {cartesian_state(parallel_seed.grasp.frame_from_grasp)},
      .grasp_link = LinkId {16},
      .link_from_grasp = {},
      .ik_initializer =
        [&](KinematicState const& state, LinkId, pose_t const&) {
          initializer_called = true;
          return std::optional<Eigen::VectorXd> {state.positions()};
        },
    },
    parallel_config, generation_config,
    inverse_kinematics_config_t {
      .max_iters = 200,
      .tol = 1e-5,
      .initialization = inverse_kinematics_initialization_e::provided,
    });
  if (sampled_joint.status != solve_status_e::success) {
    std::string detail =
      sampled_joint.failure.code + ": " + sampled_joint.failure.message;
    if (!sampled_joint.candidates.empty()) {
      detail += " / " + sampled_joint.candidates.front().grasp.failure.code +
        ": " + sampled_joint.candidates.front().grasp.failure.message;
    }
    throw std::runtime_error(detail);
  }
  require(sampled_joint.selected_candidate() != nullptr);
  require(initializer_called);
  require(sampled_joint.selected_candidate()->positions.size() == 1);
  require(sampled_joint.selected_candidate()->positions.front().size() == 6);
}
