#include <stacking_core/io/urdf.hpp>
#include <stacking_core/planner/excavator.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <numbers>
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
        "excavator closed-form requirement failed at line " +
        std::to_string(location.line()));
    }
  }

  std::filesystem::path asset(std::string const& name) {
    return std::filesystem::path {__FILE__}.parent_path().parent_path() /
      "assets" / name;
  }

  pose_t legacy_end_from_task() {
    Matrix3 const R =
      Eigen::AngleAxis<Scalar> {-std::numbers::pi / 2.0, Vector3::UnitZ()}
        .toRotationMatrix() *
      Eigen::AngleAxis<Scalar> {-std::numbers::pi / 2.0, Vector3::UnitX()}
        .toRotationMatrix();
    return pose_t {Vector3 {-0.01, 0.0, -0.535}, Quaternion {R}};
  }

  excavator_ik_chain_t make_chain(KinematicModel const& model) {
    return excavator_ik_chain_t {
      .swing = model.findJoint("upper_body_joint")->id,
      .boom = model.findJoint("boom_joint")->id,
      .arm = model.findJoint("arm_joint")->id,
      .bucket = model.findJoint("bucket_joint")->id,
      .tilt = model.findJoint("tilt_joint")->id,
      .rotate = model.findJoint("rotate_joint")->id,
      .end_link = model.findLink("cs_rotate")->id,
      .end_from_task = legacy_end_from_task(),
    };
  }

}  // namespace

int main() {
  using namespace stacking_core;

  UrdfModel const urdf = load_urdf_model(asset("excavator_kinematics.urdf"));
  KinematicModel const& model = urdf.kinematics();
  excavator_ik_chain_t const chain = make_chain(model);
  ExcavatorIkInitializer const initializer {urdf.kinematicsPtr(), chain};

  Eigen::VectorXd target_q(6), initial_q(6);
  target_q << 0.5, 0.2, 0.1, -0.3, 0.2, 0.1;
  initial_q << 0.0, 0.1, 0.1, -0.2, 0.1, 0.05;
  KinematicState target_state {FrameId {1}, urdf.kinematicsPtr()};
  target_state.setPositions(target_q);
  pose_t const frame_from_end =
    forward_kinematics(target_state).frameFromLink(chain.end_link);
  pose_t const frame_from_task = compose(frame_from_end, chain.end_from_task);

  KinematicState initial_state {FrameId {1}, urdf.kinematicsPtr()};
  initial_state.setPositions(initial_q);
  excavator_ik_seed_result_t const seed =
    initializer.seed(initial_state, frame_from_task);

  // Emitted by diffsim commit 0c433b2 before LM refinement, including its
  // revolute wrapping, wrist-sign correction, and 0.01 joint-limit margin.
  Eigen::VectorXd expected_seed(6);
  expected_seed << 0.4999999999992597, 0.26462054248924982,
    -0.091296526274079248, -0.099652921961513208, 0.19990734640810093,
    0.1000007445815001;
  require(seed.status == solve_status_e::success);
  require(seed.positions.isApprox(expected_seed, 2e-12));

  KinematicState seeded_state = initial_state;
  seeded_state.setPositions(seed.positions);
  inverse_kinematics_result_t const refined = solve_inverse_kinematics(
    inverse_kinematics_problem_t {
      .initial_state = seeded_state,
      .link = chain.end_link,
      .frame_from_link = frame_from_task,
      .position_only = false,
    },
    inverse_kinematics_config_t {
      .max_iters = 2000,
      .tol = 1e-8,
      .initialization = inverse_kinematics_initialization_e::provided,
    });
  Eigen::VectorXd expected_refined(6);
  expected_refined << 0.49849732203268404, 0.16394597144841949,
    0.090060085882365026, 0.51514612329784537, -0.54639536919100617,
    -0.89578449348498301;
  require(refined.status == solve_status_e::infeasible);
  require(refined.positions.isApprox(expected_refined, 2e-12));

  // Root poses are external state. Applying the same rigid root transform to
  // the requested task must leave the model-relative seed unchanged.
  pose_t const moved_root {
    Vector3 {3.0, -2.0, 1.0},
    Quaternion {Eigen::AngleAxis<Scalar> {0.4, Vector3::UnitZ()}},
  };
  KinematicState moved_initial = initial_state;
  moved_initial.setFrameFromRoot(model.roots().front(), moved_root);
  pose_t const moved_target = compose(moved_root, frame_from_task);
  excavator_ik_seed_result_t const moved_seed =
    initializer.seed(moved_initial, moved_target);
  require(moved_seed.status == solve_status_e::success);
  require(moved_seed.positions.isApprox(expected_seed, 2e-12));

  pose_t unreachable = frame_from_task;
  unreachable.position += 100.0 * Vector3::UnitX();
  excavator_ik_seed_result_t const failed =
    initializer.seed(initial_state, unreachable);
  require(failed.status == solve_status_e::infeasible);
  require(failed.failure.code == "closed_form_infeasible");
}
