#include <stacking_core/io/urdf.hpp>
#include <stacking_core/planner.hpp>

#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

  using namespace stacking_core;

  void require(
    bool condition,
    std::source_location location = std::source_location::current()) {
    if (!condition) {
      throw std::runtime_error(
        "inverse-kinematics requirement failed at line " +
        std::to_string(location.line()));
    }
  }

  std::filesystem::path asset(std::string const& name) {
    return std::filesystem::path {__FILE__}.parent_path() / "assets" / name;
  }

  KinematicState make_state(
    UrdfModel const& urdf, Eigen::VectorXd const& positions) {
    KinematicState state {FrameId {1}, urdf.kinematicsPtr()};
    state.setPositions(positions);
    return state;
  }

  pose_t link_pose(
    UrdfModel const& urdf, LinkId link, Eigen::VectorXd const& positions) {
    KinematicState const state = make_state(urdf, positions);
    return forward_kinematics(state).frameFromLink(link);
  }

  inverse_kinematics_result_t solve(
    UrdfModel const& urdf, LinkId link, Eigen::VectorXd const& initial,
    pose_t const& target, bool position_only) {
    return solve_inverse_kinematics(
      inverse_kinematics_problem_t {
        .initial_state = make_state(urdf, initial),
        .link = link,
        .frame_from_link = target,
        .position_only = position_only,
      },
      inverse_kinematics_config_t {
        .max_iters = 2000,
        .tol = 1e-8,
        .initialization = inverse_kinematics_initialization_e::swing,
      });
  }

  std::shared_ptr<KinematicModel const> make_mimic_model() {
    return std::make_shared<KinematicModel>(kinematic_model_config_t {
      .links =
        {
          kinematic_link_t {.id = LinkId {1}, .name = "root"},
          kinematic_link_t {.id = LinkId {2}, .name = "arm"},
          kinematic_link_t {.id = LinkId {3}, .name = "tool"},
        },
      .joints =
        {
          kinematic_joint_t {
            .id = JointId {1},
            .name = "drive",
            .type = joint_type_e::revolute,
            .parent = LinkId {1},
            .child = LinkId {2},
            .parent_from_child_zero =
              pose_t {Vector3::UnitX(), Quaternion::Identity()},
            .axis = Vector3::UnitZ(),
            .limit = joint_limit_t {.lower = -1.0, .upper = 1.0},
            .mimic = std::nullopt,
          },
          kinematic_joint_t {
            .id = JointId {2},
            .name = "limited_mimic",
            .type = joint_type_e::revolute,
            .parent = LinkId {2},
            .child = LinkId {3},
            .parent_from_child_zero =
              pose_t {Vector3::UnitX(), Quaternion::Identity()},
            .axis = Vector3::UnitZ(),
            .limit = joint_limit_t {.lower = -0.1, .upper = 0.1},
            .mimic =
              joint_mimic_t {
                .source = JointId {1},
                .multiplier = 1.0,
                .offset = 0.0,
              },
          },
        },
    });
  }

}  // namespace

int main() {
  using namespace stacking_core;

  UrdfModel const urdf = load_urdf_model(asset("excavator_kinematics.urdf"));
  KinematicModel const& model = urdf.kinematics();
  LinkId const link = model.findLink("cs_rotate")->id;

  // Emitted by diffsim commit 0c433b2 with analytic IK disabled. This target
  // places arm_joint at its upper limit; the legacy 0.01 joint margin therefore
  // makes the exact pose infeasible. Preserve both the iterate and
  // max-iteration outcome instead of weakening the parity check to pose error
  // alone.
  Eigen::VectorXd target_q(6), initial_q(6), expected_q(6);
  target_q << 0.5, 0.2, 0.1, -0.3, 0.2, 0.1;
  initial_q << 0.0, 0.1, 0.1, -0.2, 0.1, 0.05;
  expected_q << 0.49999888839216267, 0.20335755969869984, 0.090000299672603035,
    -0.29308932271391486, 0.17863852456372786, 0.091701636726408076;
  inverse_kinematics_result_t const full =
    solve(urdf, link, initial_q, link_pose(urdf, link, target_q), false);
  require(full.status == solve_status_e::max_iters);
  require(full.solver.iters == 2000);
  require(full.positions.isApprox(expected_q, 2e-12));
  require(std::abs(full.pos_error - 0.001728743204188058) < 2e-12);
  require(std::abs(full.rot_error - 0.00026300642270710162) < 2e-12);

  // Position-only IK deliberately drops the orientation rows. The different
  // terminal orientation is therefore expected, not an approximation failure.
  target_q << 0.4, 0.5, -0.7, 0.2, -0.3, 0.4;
  initial_q << -0.2, 0.0, -0.2, 0.0, 0.0, 0.0;
  expected_q << 0.40000003666866135, 0.36161118608531462, -0.38483119690734346,
    0.63462553939975974, 0.0, 0.0;
  pose_t const position_target = link_pose(urdf, link, target_q);
  inverse_kinematics_result_t const position =
    solve(urdf, link, initial_q, position_target, true);
  require(position.status == solve_status_e::success);
  require(position.solver.converged);
  require(position.solver.iters == 6);
  require(position.positions.isApprox(expected_q, 2e-12));
  require(std::abs(position.pos_error - 8.4108671453767317e-05) < 2e-12);
  require(position.rot_error == 0.0);
  require(
    position.frame_from_link.orientation.angularDistance(
      position_target.orientation) > 0.1);

  // A satisfied full-pose target exits before taking an LM step.
  inverse_kinematics_result_t const satisfied =
    solve(urdf, link, target_q, position_target, false);
  require(satisfied.status == solve_status_e::success);
  require(satisfied.solver.iters == 0);
  require(satisfied.positions.isApprox(target_q, 2e-12));
  require(satisfied.pos_error < 1e-12);
  require(satisfied.rot_error < 1e-12);

  inverse_kinematics_result_t const invalid =
    solve_inverse_kinematics(inverse_kinematics_problem_t {
      .initial_state = make_state(urdf, initial_q),
      .link = LinkId {9999},
      .frame_from_link = position_target,
      .position_only = false,
    });
  require(invalid.status == solve_status_e::invalid_problem);
  require(invalid.failure.code == "unknown_link");

  // Mimic limits constrain their source coordinate too. A pose that is exact
  // but violates the mimic joint's limit must not be reported as converged.
  std::shared_ptr<KinematicModel const> const mimic_model = make_mimic_model();
  KinematicState mimic_state {FrameId {2}, mimic_model};
  Eigen::VectorXd mimic_q(1);
  mimic_q << 0.5;
  mimic_state.setPositions(mimic_q);
  pose_t const mimic_target =
    forward_kinematics(mimic_state).frameFromLink(LinkId {3});
  inverse_kinematics_result_t const mimic =
    solve_inverse_kinematics(inverse_kinematics_problem_t {
      .initial_state = mimic_state,
      .link = LinkId {3},
      .frame_from_link = mimic_target,
      .position_only = false,
    });
  require(mimic.status == solve_status_e::infeasible);
  require(!mimic.solver.converged);
}
