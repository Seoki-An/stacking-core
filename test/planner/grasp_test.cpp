#include <stacking_core/planner.hpp>
#include <stacking_core/io/urdf.hpp>

#include "evaluation.hpp"
#include "force.hpp"

#include <source_location>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

  using namespace stacking_core;

  void require(
    bool condition,
    std::source_location location = std::source_location::current()) {
    if (!condition) {
      throw std::runtime_error(
        "grasp test requirement failed at line " +
        std::to_string(location.line()));
    }
  }

  Matrix3X cube_nodes(Scalar half_extent) {
    Matrix3X nodes(3, 8);
    nodes << -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0,
      -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0;
    return half_extent * nodes;
  }

  std::filesystem::path asset(std::string const& name) {
    return std::filesystem::path {__FILE__}.parent_path() / "assets" / name;
  }

  gripper_model_t reference_gripper() {
    UrdfModel model = load_urdf_model(asset("grasp_reference.urdf"));
    KinematicState state {FrameId {1}, model.kinematicsPtr()};
    LinkId const root = model.kinematics().findLink("root")->id;
    LinkId const left = model.kinematics().findLink("left_pad")->id;
    LinkId const right = model.kinematics().findLink("right_pad")->id;
    Eigen::VectorXd opening_offset = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd opening_direction = Eigen::VectorXd::Ones(2);
    GeometryId const left_geometry = model.bodyModel(left).geometry(0).id();
    GeometryId const right_geometry = model.bodyModel(right).geometry(0).id();
    return gripper_model_t {
      .state = std::move(state),
      .root_link = root,
      .grasp_from_root = {},
      .opening_offset = std::move(opening_offset),
      .opening_direction = std::move(opening_direction),
      .opening_lower = -0.1,
      .opening_upper = 0.1,
      .collision_bodies =
        {
          grasp_link_body_t {left, EntityId {201}, model.bodyModelPtr(left)},
          grasp_link_body_t {right, EntityId {202}, model.bodyModelPtr(right)},
        },
      .contact_geometries =
        {
          grasp_contact_geometry_t {
            EntityId {201}, left_geometry, grasp_contact_side_e::left},
          grasp_contact_geometry_t {
            EntityId {202}, right_geometry, grasp_contact_side_e::right},
        },
    };
  }

  std::shared_ptr<BodyModel const> cube_model(
    BodyModelId model, GeometryId geometry, Scalar half_extent) {
    std::vector<geometry_config_t> geometries;
    geometries.emplace_back(dsf_vert_geometry_config_t {
      .properties =
        geometry_properties_t {
          .id = geometry,
          .body_from_geometry = {},
          .material = {},
        },
      .nodes = cube_nodes(half_extent),
      .sharpness = 20,
    });
    return std::make_shared<BodyModel>(body_model_config_t {
      .id = model,
      .inertial = std::nullopt,
      .geometries = std::move(geometries),
    });
  }

  std::shared_ptr<KinematicModel const> hand_kinematics() {
    return std::make_shared<KinematicModel>(kinematic_model_config_t {
      .links =
        {
          kinematic_link_t {.id = LinkId {1}, .name = "root"},
          kinematic_link_t {.id = LinkId {2}, .name = "left"},
          kinematic_link_t {.id = LinkId {3}, .name = "right"},
        },
      .joints =
        {
          kinematic_joint_t {
            .id = JointId {1},
            .name = "left_opening",
            .type = joint_type_e::prismatic,
            .parent = LinkId {1},
            .child = LinkId {2},
            .parent_from_child_zero =
              pose_t {Vector3 {1.1, 0.0, 0.0}, Quaternion::Identity()},
            .axis = Vector3::UnitX(),
            .limit = joint_limit_t {-0.1, 0.1},
            .mimic = std::nullopt,
          },
          kinematic_joint_t {
            .id = JointId {2},
            .name = "right_opening",
            .type = joint_type_e::prismatic,
            .parent = LinkId {1},
            .child = LinkId {3},
            .parent_from_child_zero =
              pose_t {Vector3 {-1.1, 0.0, 0.0}, Quaternion::Identity()},
            .axis = -Vector3::UnitX(),
            .limit = joint_limit_t {-0.1, 0.1},
            .mimic = std::nullopt,
          },
        },
    });
  }

  gripper_model_t gripper(FrameId frame) {
    auto kinematics = hand_kinematics();
    KinematicState state {frame, kinematics};
    state.setFrameFromRoot(LinkId {1}, pose_t {});
    auto left = cube_model(BodyModelId {11}, GeometryId {11}, 0.1);
    auto right = cube_model(BodyModelId {12}, GeometryId {12}, 0.1);
    Eigen::VectorXd opening_offset = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd opening_direction(2);
    opening_direction << 0.5, 0.5;
    return gripper_model_t {
      .state = std::move(state),
      .root_link = LinkId {1},
      .grasp_from_root = {},
      .opening_offset = std::move(opening_offset),
      .opening_direction = std::move(opening_direction),
      .opening_lower = -0.1,
      .opening_upper = 0.1,
      .collision_bodies =
        {
          grasp_link_body_t {LinkId {2}, EntityId {101}, left},
          grasp_link_body_t {LinkId {3}, EntityId {102}, right},
        },
      .contact_geometries =
        {
          grasp_contact_geometry_t {
            EntityId {101}, GeometryId {11}, grasp_contact_side_e::left},
          grasp_contact_geometry_t {
            EntityId {102}, GeometryId {12}, grasp_contact_side_e::right},
        },
    };
  }

  phase_scene_t phase(
    std::shared_ptr<BodyModel const> const& target_model, EntityId target,
    Scalar target_x) {
    std::vector<BodyInstance> bodies;
    bodies.emplace_back(body_instance_config_t {
      .id = target,
      .model = target_model,
      .frame_from_body =
        pose_t {Vector3 {target_x, 0.0, 0.0}, Quaternion::Identity()},
      .motion = {},
      .mobility = mobility_e::static_body,
    });
    auto snapshot = std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
      .frame = FrameId {1},
      .bodies = std::move(bodies),
    });
    return phase_scene_t {SceneView {snapshot, {target}}, target};
  }

  grasp_generation_config_t test_config() {
    grasp_generation_config_t config;
    config.cost = grasp_cost_weights_t {};
    config.cost.antipodal_normal = 0.0;
    config.cost.antipodal_position = 0.0;
    config.cost.align = 0.0;
    config.cost.enclosure = 0.0;
    config.cost.radial_distance = 0.0;
    config.cost.center_distance = 0.0;
    config.cost.contact = 0.0;
    config.separate_margin = -0.1;
    config.trust_region.max_iters = 2;
    config.alm.max_iters = 2;
    return config;
  }

}  // namespace

int main() {
  using namespace stacking_core;

  std::vector<grasp_detail::force_contact_t> force_contacts(2);
  force_contacts[0].feature.gap = 0.03;
  force_contacts[0].feature.point_second = Vector3 {0.2, -0.1, 0.3};
  force_contacts[0].feature.normal = Vector3 {0.2, 0.1, 0.97}.normalized();
  force_contacts[1].feature.gap = -0.02;
  force_contacts[1].feature.point_second = Vector3 {-0.25, 0.15, 0.35};
  force_contacts[1].feature.normal = Vector3 {-0.1, 0.3, 0.95}.normalized();
  force_contacts[0].feature.d_gap.leftCols<6>() << 0.1, -0.2, 0.3, 0.05, -0.04,
    0.02;
  force_contacts[1].feature.d_gap.leftCols<6>() << -0.15, 0.25, -0.05, -0.03,
    0.06, 0.01;
  force_contacts[0].feature.d_point_second.leftCols<6>() << 1.0, 0.0, 0.0, 0.0,
    0.3, 0.1, 0.0, 1.0, 0.0, -0.3, 0.0, 0.2, 0.0, 0.0, 1.0, -0.1, -0.2, 0.0;
  force_contacts[1].feature.d_point_second.leftCols<6>() << 1.0, 0.0, 0.0, 0.0,
    0.35, -0.15, 0.0, 1.0, 0.0, -0.35, 0.0, -0.25, 0.0, 0.0, 1.0, 0.15, 0.25,
    0.0;
  force_contacts[0].feature.d_normal.leftCols<6>() << 0.0, 0.0, 0.0, 0.01, 0.2,
    -0.03, 0.0, 0.0, 0.0, -0.2, 0.02, 0.04, 0.0, 0.0, 0.0, 0.03, -0.04, 0.01;
  force_contacts[1].feature.d_normal.leftCols<6>() << 0.0, 0.0, 0.0, -0.02,
    0.15, 0.05, 0.0, 0.0, 0.0, -0.16, -0.01, 0.02, 0.0, 0.0, 0.0, -0.04, -0.03,
    0.02;
  force_contacts[0].body_jac.leftCols<6>().setIdentity();
  force_contacts[1].body_jac.leftCols<6>().setIdentity();
  force_contacts[0].body_jac.col(6) << 0.1, -0.05, 0.02, 0.03, 0.01, -0.02;
  force_contacts[1].body_jac.col(6) << -0.08, 0.04, -0.01, 0.02, -0.03, 0.01;

  grasp_force_config_t force_config;
  force_config.weight = grasp_force_weights_t {
    .wrench = 1.2,
    .complementarity = 0.7,
    .cone = 1.4,
    .moment = 3.5,
  };
  force_config.friction = 0.8;
  Eigen::VectorXd forces(6);
  forces << 0.4, -0.2, 1.1, -0.3, 0.5, 0.7;
  Vector6 wrench;
  wrench << 0.2, -0.1, -1.3, 0.05, -0.04, 0.02;
  Vector3 const center {0.01, -0.02, 0.04};
  auto const force_eval = grasp_detail::evaluate_contact_forces(
    forces, force_contacts, wrench, center, force_config);
  require(force_eval.finite());
  require(std::abs(force_eval.cost - 0.24503485) < 1e-12);
  Eigen::VectorXd expected_force_grad(6);
  expected_force_grad << 0.27726, 0.212028, 0.652773, 0.333414, 0.339764,
    0.523336;
  require(force_eval.grad.isApprox(expected_force_grad, 1e-12));
  Eigen::MatrixXd expected_force_hess(6, 6);
  expected_force_hess << 1.51143, 0.06384, -0.20748, 1.4814, -0.08736, 0.28392,
    0.06384, 1.63617, 0.08736, -0.13566, 1.33104, -0.18564, -0.20748, 0.08736,
    1.37913, -0.24738, 0.10416, 0.9354, 1.4814, -0.13566, -0.24738, 1.72528,
    0.18564, 0.33852, -0.08736, 1.33104, 0.10416, 0.18564, 1.88782, -0.22134,
    0.28392, -0.18564, 0.9354, 0.33852, -0.22134, 1.60558;
  require(force_eval.hess.isApprox(expected_force_hess, 1e-12));

  auto const force_refinement =
    grasp_detail::refine_contact_forces(force_contacts, center, force_config);
  require(force_refinement.finite());
  require(std::abs(force_refinement.cost - 0.74947721991706717) < 1e-11);
  grasp_detail::vector7_t expected_refinement_grad;
  expected_refinement_grad << 1.887118063014896, -1.1763174461337691,
    0.24903264705471073, 0.37325778626351969, 0.54661106617856536,
    0.17039698062548808, 0.082211412930657854;
  require(force_refinement.grad.isApprox(expected_refinement_grad, 1e-10));
  require(force_refinement.contact_forces.size() == 2);
  require(force_refinement.contact_forces[0].isApprox(
    Vector3 {0.084497796180167895, 0.24268760013302593, -0.047800202913884308},
    1e-11));
  require(force_refinement.contact_forces[1].isApprox(
    Vector3 {-0.14039182287024268, 0.47185680968117494, 0.20537585970798622},
    1e-11));

  auto target_model = cube_model(BodyModelId {1}, GeometryId {1}, 1.0);
  phase_scene_t pick = phase(target_model, EntityId {1}, 0.0);
  grasp_problem_t pose_problem {
    .phases = {pick},
    .gripper = gripper(FrameId {1}),
    .seed =
      grasp_t {
        .frame_from_grasp = pose_t {},
        .opening = 0.0,
      },
  };
  grasp_result_t const pose_result =
    solve_grasp_pose(pose_problem, test_config());
  require(pose_result.status == solve_status_e::success);
  require(pose_result.selected_candidate() != nullptr);
  require(pose_result.selected_candidate()->contacts.size() == 2);
  require(pose_result.selected_candidate()->solver.converged);
  require(std::abs(pose_result.selected_candidate()->grasp.opening) < 1e-12);

  grasp_generation_config_t derivative_config;
  derivative_config.separate_margin = -0.1;
  grasp_t derivative_grasp {
    .frame_from_grasp =
      pose_t {
        Vector3 {0.03, -0.04, 0.02},
        Quaternion {Eigen::AngleAxis<Scalar> {
          0.08, Vector3 {1.0, -2.0, 0.5}.normalized()}},
      },
    .opening = -0.01,
  };
  grasp_detail::grasp_evaluation_t const derivative_eval =
    grasp_detail::evaluate_grasp(
      pose_problem, derivative_config, derivative_grasp);
  require(derivative_eval.finite());
  constexpr Scalar h = 1e-6;
  for (Eigen::Index axis = 0; axis < 7; ++axis) {
    grasp_detail::vector7_t step = grasp_detail::vector7_t::Zero();
    step(axis) = h;
    auto const plus = grasp_detail::evaluate_grasp(
      pose_problem, derivative_config,
      grasp_detail::apply_grasp_step(derivative_grasp, step));
    step(axis) = -h;
    auto const minus = grasp_detail::evaluate_grasp(
      pose_problem, derivative_config,
      grasp_detail::apply_grasp_step(derivative_grasp, step));
    Scalar const objective_fd = (plus.objective - minus.objective) / (2.0 * h);
    require(std::abs(objective_fd - derivative_eval.grad(axis)) < 2e-4);
    Eigen::VectorXd const constraints_fd =
      (plus.c_ineq - minus.c_ineq) / (2.0 * h);
    require(
      (constraints_fd - derivative_eval.jac_ineq.col(axis)).norm() < 2e-4);
  }

  grasp_problem_t reference_problem {
    .phases = {pick},
    .gripper = reference_gripper(),
    .seed = derivative_grasp,
  };
  auto const reference_eval = grasp_detail::evaluate_grasp(
    reference_problem, grasp_generation_config_t {}, derivative_grasp);
  // Emitted by legacy diffsim commit 0c433b2 from objective_grasp using the
  // same URDF, target nodes, grasp, and default weights.
  require(std::abs(reference_eval.objective - (-5.9726995481833871)) < 5e-8);
  require(std::abs(reference_eval.score - 0.19099802753562986) < 5e-8);
  grasp_detail::vector7_t expected_grad;
  expected_grad << 8.0691882273050872, 9.8609435551485731, 6.2800602904401384,
    0.41584708318916785, -0.77584320311076294, -0.27560599091035365,
    -0.34865820775584222;
  require(reference_eval.grad.isApprox(expected_grad, 5e-7));
  Eigen::VectorXd expected_c_ineq(6);
  expected_c_ineq << 0.074908889054103187, 0.13504775031467306,
    -0.065908889054103192, -0.12604775031467305, -0.11, -0.090000000000000011;
  require(reference_eval.c_ineq.isApprox(expected_c_ineq, 5e-8));
  grasp_detail::matrix_x7_t expected_jac(6, 7);
  expected_jac << -0.99996178199551033, -0.00030035327660423507,
    -0.0087375240106787409, 0.00046706664903572613, -0.14540651730778603,
    0.050894497101583433, -0.99798980763919354, 0.9999690458268341,
    0.0039524264493334118, 0.0068033604441738215, 0.00042596346596856225,
    -0.14882931145869194, 0.041066767339279789, -0.99792078358373904,
    0.99996178199551033, 0.00030035327660423507, 0.0087375240106787409,
    -0.00046706664903572613, 0.14540651730778603, -0.050894497101583433,
    0.99798980763919354, -0.9999690458268341, -0.0039524264493334118,
    -0.0068033604441738215, -0.00042596346596856225, 0.14882931145869194,
    -0.041066767339279789, 0.99792078358373904, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0;
  require(reference_eval.jac_ineq.isApprox(expected_jac, 5e-6));

  grasp_generation_config_t bilevel_config;
  bilevel_config.force.enabled = true;
  auto const bilevel_eval = grasp_detail::evaluate_grasp(
    reference_problem, bilevel_config, derivative_grasp);
  require(bilevel_eval.finite());
  require(bilevel_eval.objective > reference_eval.objective);
  require(bilevel_eval.contacts.size() == 2);
  require(bilevel_eval.contacts[0].force.has_value());
  require(bilevel_eval.contacts[1].force.has_value());
  require(bilevel_eval.contacts[0].force->allFinite());
  require(bilevel_eval.contacts[1].force->allFinite());

  auto sliding_model =
    std::make_shared<KinematicModel>(kinematic_model_config_t {
      .links =
        {
          kinematic_link_t {.id = LinkId {30}, .name = "base"},
          kinematic_link_t {.id = LinkId {31}, .name = "sliding_tool"},
        },
      .joints = {kinematic_joint_t {
        .id = JointId {30},
        .name = "tool_x",
        .type = joint_type_e::prismatic,
        .parent = LinkId {30},
        .child = LinkId {31},
        .parent_from_child_zero = {},
        .axis = Vector3::UnitX(),
        .limit = joint_limit_t {-0.5, 0.5},
        .mimic = std::nullopt,
      }},
    });
  KinematicState sliding_state {FrameId {1}, sliding_model};
  sliding_state.setFrameFromRoot(LinkId {30}, pose_t {});
  Eigen::VectorXd sliding_q(1);
  sliding_q << 0.03;
  sliding_state.setPositions(sliding_q);
  joint_grasp_problem_t derivative_joint_problem {
    .grasp =
      grasp_problem_t {
        .phases = {pick},
        .gripper = gripper(FrameId {1}),
        .seed = grasp_t {.frame_from_grasp = {}, .opening = -0.01},
      },
    .initial_states = {sliding_state},
    .grasp_link = LinkId {31},
    .link_from_grasp = {},
  };
  Eigen::VectorXd joint_variables(2);
  joint_variables << 0.03, -0.01;
  grasp_detail::joint_evaluation_t const joint_eval =
    grasp_detail::evaluate_joint(
      derivative_joint_problem, derivative_config, joint_variables);
  require(joint_eval.finite());
  for (Eigen::Index axis = 0; axis < joint_variables.size(); ++axis) {
    Eigen::VectorXd plus_variables = joint_variables;
    Eigen::VectorXd minus_variables = joint_variables;
    plus_variables(axis) += h;
    minus_variables(axis) -= h;
    auto const plus = grasp_detail::evaluate_joint(
      derivative_joint_problem, derivative_config, plus_variables);
    auto const minus = grasp_detail::evaluate_joint(
      derivative_joint_problem, derivative_config, minus_variables);
    Scalar const objective_fd = (plus.objective - minus.objective) / (2.0 * h);
    require(std::abs(objective_fd - joint_eval.grad(axis)) < 2e-4);
    Eigen::VectorXd const constraints_fd =
      (plus.c_ineq - minus.c_ineq) / (2.0 * h);
    require((constraints_fd - joint_eval.jac_ineq.col(axis)).norm() < 2e-4);
  }

  auto manipulator = std::make_shared<KinematicModel>(kinematic_model_config_t {
    .links = {kinematic_link_t {.id = LinkId {20}, .name = "tool"}},
    .joints = {},
  });
  KinematicState pick_state {FrameId {1}, manipulator};
  pick_state.setFrameFromRoot(LinkId {20}, pose_t {});
  KinematicState place_state {FrameId {1}, manipulator};
  place_state.setFrameFromRoot(
    LinkId {20}, pose_t {Vector3 {1.0, 0.0, 0.0}, Quaternion::Identity()});
  phase_scene_t place = phase(target_model, EntityId {2}, 1.0);
  joint_grasp_problem_t joint_problem {
    .grasp =
      grasp_problem_t {
        .phases = {pick, place},
        .gripper = gripper(FrameId {1}),
        .seed =
          grasp_t {
            .frame_from_grasp = {},
            .opening = 0.0,
          },
      },
    .initial_states = {pick_state, place_state},
    .grasp_link = LinkId {20},
    .link_from_grasp = {},
  };
  joint_grasp_result_t const joint_result =
    solve_joint_grasp(joint_problem, test_config());
  require(joint_result.status == solve_status_e::success);
  require(joint_result.selected_candidate() != nullptr);
  require(joint_result.selected_candidate()->positions.size() == 2);
  require(joint_result.selected_candidate()->positions[0].size() == 0);
  require(joint_result.selected_candidate()->grasp.contacts.size() == 2);
  require(joint_result.selected_candidate()->grasp.solver.converged);

  joint_problem.initial_states.pop_back();
  joint_grasp_result_t const invalid =
    solve_joint_grasp(joint_problem, test_config());
  require(invalid.status == solve_status_e::invalid_problem);
  require(invalid.failure.code == "joint_phase_count_mismatch");
}
