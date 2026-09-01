#include <stacking_core/io/urdf.hpp>
#include <stacking_core/kinematics.hpp>
#include <stacking_core/planner.hpp>

#include <array>
#include <cmath>
#include <filesystem>
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
        "planner diffsim-reference requirement failed at line " +
        std::to_string(location.line()));
    }
  }

  std::filesystem::path asset(std::string const& name) {
    return std::filesystem::path {__FILE__}.parent_path() / "assets" / name;
  }

  Matrix6X legacy_body_jacobian() {
    Matrix6X jac(6, 6);
    jac << -0.87769056442814442, -8.9642849425970841, -3.352142943128849,
      -0.54271239175255859, -2.7709522121497396e-18, 0.0, -8.7918002453863462,
      1.2669323169655162, 0.56050345060659867, 0.14825298115311777,
      -2.7616754795621317e-17, 0.0, 1.7291392736580351, 1.8047211336709832,
      1.10083866863895, 0.46063590589194314, 1.1809740769599698e-17, 0.0,
      -0.99376435710351163, 0.097845958371046135, 0.097845958371046135,
      0.097845958371046135, 0.99500416523733659, 0.0, 0.085020112839913675,
      0.97518856562848688, 0.97518856562848688, 0.97518856562848688,
      -0.099833416984790035, 0.0, -0.072138637104771197, -0.19857852325442066,
      -0.19857852325442066, -0.19857852325442066, 3.6732050873178324e-06, 1.0;
    return jac;
  }

  std::array<stable_pose_t, 8> legacy_stable_poses() {
    // Emitted by diffsim commit 0c433b2 for the copied model_2.obj fixture with
    // local_com=0, sampling_level=3, and the default stable-pose configuration.
    return {{
      stable_pose_t {
        .resting_dir_body =
          Vector3 {
            0.26835239635016706, 0.070838120289745801, 0.96071273130261903},
        .cone_angle = 0.7137575638110526,
        .support_point_body =
          Vector3 {
            0.21018940182287052, -0.010667396387342154, 0.51842736826912805},
      },
      stable_pose_t {
        .resting_dir_body =
          Vector3 {
            -0.008805212731749423, -0.44617999550504794, -0.89489992727671186},
        .cone_angle = 0.32300233369655507,
        .support_point_body =
          Vector3 {
            0.064943796733839682, -0.32008917522204555, -0.52885729253555935},
      },
      stable_pose_t {
        .resting_dir_body =
          Vector3 {
            0.4977263982955204, 0.6120918801623606, -0.61450139355340405},
        .cone_angle = 0.23557566530579988,
        .support_point_body =
          Vector3 {
            0.44471691424028553, 0.41283351120169332, -0.44454043335332993},
      },
      stable_pose_t {
        .resting_dir_body =
          Vector3 {
            -0.13191631054997233, -0.90013033384685359, -0.41516679672107099},
        .cone_angle = 0.45669199389985937,
        .support_point_body =
          Vector3 {
            -0.0095637385593958513, -0.59268546156709145, -0.23372022593229541},
      },
      stable_pose_t {
        .resting_dir_body =
          Vector3 {
            -0.60458034530165627, 0.72381434078769602, -0.33252880498537757},
        .cone_angle = 0.36692954866116295,
        .support_point_body =
          Vector3 {
            -0.39036796515390526, 0.50395669758247597, -0.23569207162824851},
      },
      stable_pose_t {
        .resting_dir_body =
          Vector3 {
            0.34943873292136857, 0.88912732795687399, 0.29554215708520831},
        .cone_angle = 0.1465430492863127,
        .support_point_body =
          Vector3 {
            0.34513629274096302, 0.65166196737332049, 0.25012249246970097},
      },
      stable_pose_t {
        .resting_dir_body =
          Vector3 {
            -0.87209992251070878, -0.088576875732283372, 0.48124407761792104},
        .cone_angle = 0.17548485911780512,
        .support_point_body =
          Vector3 {
            -0.66537949436600174, -0.12231211853381993, 0.42367058206113717},
      },
      stable_pose_t {
        .resting_dir_body =
          Vector3 {
            0.70746964228075981, -0.6879262245219202, -0.16200066316577375},
        .cone_angle = 0.1572395619205128,
        .support_point_body =
          Vector3 {
            0.60941767287699677, -0.57179289180079762, -0.10575103208016967},
      },
    }};
  }

}  // namespace

int main() {
  using namespace stacking_core;

  // FK and the raw hybrid body Jacobian were emitted by diffsim commit
  // 0c433b2 using assets/excavator/vdk23_cx.urdf, end link cs_rotate, and
  // q=[0.5, 0.2, 0.1, -0.3, 0.2, 0.1]. The reduced URDF preserves that exact
  // kinematic chain while omitting presentation-only mesh links.
  UrdfModel model = load_urdf_model(asset("excavator_kinematics.urdf"));
  KinematicModel const& kinematics = model.kinematics();
  KinematicState state {FrameId {1}, model.kinematicsPtr()};
  state.setFrameFromRoot(kinematics.roots().front(), pose_t {});
  Eigen::VectorXd q(6);
  q << 0.5, 0.2, 0.1, -0.3, 0.2, 0.1;
  state.setPositions(q);

  KinematicSnapshot const snapshot = forward_kinematics(state);
  LinkId const end = kinematics.findLink("cs_rotate")->id;
  pose_t const& actual_pose = snapshot.frameFromLink(end);
  pose_t const expected_pose {
    Vector3 {7.9295636433010186, 4.2635707260706734, 4.3072225665236612},
    Quaternion {
      -0.13027782342622959, -0.72797174116360985, -0.078271722988625281,
      0.66854945230023077},
  };
  require(actual_pose.position.isApprox(expected_pose.position, 2e-12));
  require(
    actual_pose.orientation.angularDistance(expected_pose.orientation) < 2e-12);

  // Legacy planner rows are [linear in the end-link frame; angular in the
  // end-link frame]. Core stores a conventional scene-frame geometric
  // Jacobian, so rotate both blocks before comparison.
  Matrix6X expected_jac = legacy_body_jacobian();
  Matrix3 const R = expected_pose.orientation.toRotationMatrix();
  expected_jac.topRows<3>() = R * expected_jac.topRows<3>();
  expected_jac.bottomRows<3>() = R * expected_jac.bottomRows<3>();
  require(snapshot.linkJacobian(end).isApprox(expected_jac, 2e-12));

  UrdfModel stable_model = load_urdf_model(asset("stable_pose.urdf"));
  LinkId const target = stable_model.kinematics().findLink("target")->id;
  auto const& stable_geometry = static_cast<DsfVertGeometry const&>(
    stable_model.bodyModel(target).geometry(0));
  require(stable_geometry.sharpness() == 60);
  stable_pose_result_t const stable_result = solve_stable_poses(
    stable_pose_problem_t {
      .body_model = stable_model.bodyModelPtr(target),
      .com_offset_body = Vector3::Zero(),
    },
    stable_pose_config_t {
      .sampling_level = 3,
      .stable_eigenvalue_min = -0.2,
    });
  require(stable_result.status == solve_status_e::success);
  std::array<stable_pose_t, 8> const expected_stable = legacy_stable_poses();
  require(stable_result.poses.size() == expected_stable.size());
  for (std::size_t i = 0; i < expected_stable.size(); ++i) {
    stable_pose_t const& actual = stable_result.poses[i];
    stable_pose_t const& expected = expected_stable[i];
    // Trust-region stopping can differ by a few last bits across the
    // refactor's equivalent Eigen expressions. These bounds remain far below
    // planner scale while detecting support or basin-algorithm changes.
    require(
      (actual.resting_dir_body - expected.resting_dir_body).norm() < 5e-10);
    require(std::abs(actual.cone_angle - expected.cone_angle) < 5e-10);
    require(
      (actual.support_point_body - expected.support_point_body).norm() < 1e-8);
  }

  stable_pose_result_t const missing_body =
    solve_stable_poses(stable_pose_problem_t {});
  require(missing_body.status == solve_status_e::invalid_problem);
  require(missing_body.failure.code == "missing_body_model");
}
