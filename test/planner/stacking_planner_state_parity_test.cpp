#include <stacking_core/io/urdf.hpp>
#include <stacking_core/kinematics.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

  using namespace stacking_core;

  enum class stage_role_e {
    free,
    attached,
  };

  struct sample_t {
    Eigen::VectorXd q;
    pose_t frame_from_target;
  };

  struct stage_t {
    stage_role_e role = stage_role_e::free;
    std::vector<sample_t> samples;
  };

  struct case_t {
    int index = -1;
    std::string mode;
    int stone_id = -1;
    Scalar score = 0.0;
    std::vector<stage_t> stages;
  };

  struct fixture_t {
    std::string source;
    std::string archive_sha256;
    Eigen::VectorXd q_home;
    std::vector<case_t> cases;
  };

  [[noreturn]] void fail(std::string const& message) {
    throw std::runtime_error("stacking-planner parity: " + message);
  }

  void expect_token(std::istream& stream, std::string const& expected) {
    std::string actual;
    if (!(stream >> actual) || actual != expected) {
      fail("expected token '" + expected + "', got '" + actual + "'");
    }
  }

  Eigen::VectorXd read_vector(std::istream& stream, std::size_t size) {
    Eigen::VectorXd out(static_cast<Eigen::Index>(size));
    for (std::size_t i = 0; i < size; ++i) {
      if (
        !(stream >> out(static_cast<Eigen::Index>(i))) ||
        !std::isfinite(out(static_cast<Eigen::Index>(i)))) {
        fail("invalid vector value");
      }
    }
    return out;
  }

  pose_t read_pose(std::istream& stream) {
    Eigen::Matrix4d T;
    for (Eigen::Index row = 0; row < T.rows(); ++row) {
      for (Eigen::Index col = 0; col < T.cols(); ++col) {
        if (!(stream >> T(row, col)) || !std::isfinite(T(row, col))) {
          fail("invalid pose value");
        }
      }
    }
    if (!T.bottomRows<1>().isApprox(
          Eigen::RowVector4d {0.0, 0.0, 0.0, 1.0}, 1e-12)) {
      fail("pose has invalid homogeneous row");
    }
    Matrix3 const R = T.topLeftCorner<3, 3>();
    if (
      !(R.transpose() * R).isApprox(Matrix3::Identity(), 2e-10) ||
      std::abs(R.determinant() - 1.0) > 2e-10) {
      fail("pose has invalid rotation");
    }
    return pose_t {T.topRightCorner<3, 1>(), Quaternion {R}};
  }

  fixture_t read_fixture(std::filesystem::path const& path) {
    std::ifstream stream(path);
    if (!stream) {
      fail("cannot open fixture " + path.string());
    }

    fixture_t out;
    expect_token(stream, "stacking_planner_plan_state");
    int version = 0;
    if (!(stream >> version) || version != 1) {
      fail("unsupported fixture version");
    }
    expect_token(stream, "source");
    stream >> out.source;
    expect_token(stream, "archive_sha256");
    stream >> out.archive_sha256;
    expect_token(stream, "dof");
    std::size_t dof = 0;
    stream >> dof;
    expect_token(stream, "q_home");
    out.q_home = read_vector(stream, dof);
    expect_token(stream, "case_count");
    std::size_t case_count = 0;
    stream >> case_count;

    out.cases.reserve(case_count);
    for (std::size_t case_number = 0; case_number < case_count; ++case_number) {
      expect_token(stream, "case");
      case_t value;
      std::size_t stage_count = 0;
      if (!(stream >> value.index >> value.mode >> value.stone_id >>
            value.score >> stage_count)) {
        fail("invalid case header");
      }
      value.stages.reserve(stage_count);
      for (std::size_t stage_number = 0; stage_number < stage_count;
           ++stage_number) {
        expect_token(stream, "stage");
        std::size_t stage_index = 0;
        std::string role;
        std::size_t sample_count = 0;
        if (
          !(stream >> stage_index >> role >> sample_count) ||
          stage_index != stage_number) {
          fail("invalid stage header");
        }
        stage_t stage;
        if (role == "free") {
          stage.role = stage_role_e::free;
        } else if (role == "attached") {
          stage.role = stage_role_e::attached;
        } else {
          fail("unknown stage role " + role);
        }
        stage.samples.reserve(sample_count);
        for (std::size_t i = 0; i < sample_count; ++i) {
          expect_token(stream, "sample");
          stage.samples.push_back(sample_t {
            .q = read_vector(stream, dof),
            .frame_from_target = read_pose(stream),
          });
        }
        expect_token(stream, "end_stage");
        value.stages.push_back(std::move(stage));
      }
      expect_token(stream, "end_case");
      out.cases.push_back(std::move(value));
    }
    expect_token(stream, "end_fixture");
    return out;
  }

  Scalar position_error(pose_t const& lhs, pose_t const& rhs) {
    return (lhs.position - rhs.position).norm();
  }

  Scalar rotation_error(pose_t const& lhs, pose_t const& rhs) {
    return lhs.orientation.angularDistance(rhs.orientation);
  }

  Scalar joint_distance(
    KinematicModel const& model, Eigen::VectorXd const& lhs,
    Eigen::VectorXd const& rhs) {
    Eigen::VectorXd diff = lhs - rhs;
    for (std::size_t i = 0; i < model.degreeOfFreedomCount(); ++i) {
      kinematic_joint_t const& joint =
        model.joint(model.degreeOfFreedomJoint(i));
      if (
        joint.type == joint_type_e::revolute && joint.limit.has_value() &&
        joint.limit->upper - joint.limit->lower >=
          2.0 * std::numbers::pi - 1e-12) {
        diff(static_cast<Eigen::Index>(i)) = std::remainder(
          diff(static_cast<Eigen::Index>(i)), 2.0 * std::numbers::pi);
      }
    }
    return diff.norm();
  }

  std::filesystem::path asset(std::string const& name) {
    return std::filesystem::path {__FILE__}.parent_path() / "assets" / name;
  }

}  // namespace

int main(int argc, char** argv) {
  using namespace stacking_core;

  if (argc > 3) {
    fail("usage: parity-test [fixture [manipulator.urdf]]");
  }
  bool const uses_committed_fixture = argc == 1;
  std::filesystem::path const fixture_path = argc >= 2
    ? std::filesystem::path {argv[1]}
    : asset("stacking_planner_plan_state.txt");
  std::filesystem::path const urdf_path = argc >= 3
    ? std::filesystem::path {argv[2]}
    : asset("stacking_planner_excavator_kinematics.urdf");
  fixture_t const fixture = read_fixture(fixture_path);
  if (fixture.cases.empty() || fixture.q_home.size() == 0) {
    fail("fixture must contain at least one case and one joint");
  }

  UrdfModel urdf = load_urdf_model(urdf_path);
  KinematicModel const& model = urdf.kinematics();
  if (
    model.degreeOfFreedomCount() !=
    static_cast<std::size_t>(fixture.q_home.size())) {
    fail("fixture and URDF DoF counts differ");
  }
  auto const* end_link = model.findLink("grip_body");
  if (end_link == nullptr) {
    fail("URDF has no grip_body end effector");
  }

  KinematicState state {FrameId {1}, urdf.kinematicsPtr()};
  state.setFrameFromRoot(model.roots().front(), pose_t {});

  bool saw_direct = false;
  bool saw_regrasp = false;
  Scalar max_position_error = 0.0;
  Scalar max_rotation_error = 0.0;
  std::size_t attached_sample_count = 0;

  for (case_t const& value : fixture.cases) {
    if (value.mode != "direct" && value.mode != "regrasp") {
      fail("case " + std::to_string(value.index) + " has unknown mode");
    }
    std::size_t const expected_stages = value.mode == "direct" ? 4 : 8;
    saw_direct |= value.mode == "direct";
    saw_regrasp |= value.mode == "regrasp";
    if (value.stages.size() != expected_stages || !std::isfinite(value.score)) {
      fail("case " + std::to_string(value.index) + " has invalid metadata");
    }
    if (
      (value.stages.front().samples.front().q - fixture.q_home).norm() >
        2e-12 ||
      (value.stages.back().samples.back().q - fixture.q_home).norm() > 2e-12) {
      fail("case " + std::to_string(value.index) + " is not home-bounded");
    }

    for (std::size_t stage_index = 0; stage_index < value.stages.size();
         ++stage_index) {
      stage_t const& stage = value.stages[stage_index];
      if (stage.samples.empty()) {
        fail("empty stage");
      }
      if (
        stage_index > 0 &&
        joint_distance(
          model, value.stages[stage_index - 1].samples.back().q,
          stage.samples.front().q) > 2e-12) {
        fail(
          "case " + std::to_string(value.index) + " stage " +
          std::to_string(stage_index) + " breaks joint continuity");
      }

      if (stage.role == stage_role_e::free) {
        pose_t const& fixed_target = stage.samples.front().frame_from_target;
        for (sample_t const& sample : stage.samples) {
          if (
            position_error(sample.frame_from_target, fixed_target) > 2e-12 ||
            rotation_error(sample.frame_from_target, fixed_target) > 2e-12) {
            fail("legacy free-motion target path is not static");
          }
        }
        continue;
      }

      sample_t const& first = stage.samples.front();
      state.setPositions(first.q);
      pose_t const first_link =
        forward_kinematics(state).frameFromLink(end_link->id);
      pose_t const link_from_target =
        compose(inverse(first_link), first.frame_from_target);

      for (std::size_t sample_index = 0; sample_index < stage.samples.size();
           ++sample_index) {
        sample_t const& sample = stage.samples[sample_index];
        state.setPositions(sample.q);
        pose_t const frame_from_link =
          forward_kinematics(state).frameFromLink(end_link->id);
        pose_t const actual = compose(frame_from_link, link_from_target);
        Scalar const pos_err = position_error(actual, sample.frame_from_target);
        Scalar const rot_err = rotation_error(actual, sample.frame_from_target);
        max_position_error = std::max(max_position_error, pos_err);
        max_rotation_error = std::max(max_rotation_error, rot_err);
        ++attached_sample_count;
        if (pos_err > 2e-10 || rot_err > 2e-10) {
          std::ostringstream message;
          message << "case " << value.index << " stage " << stage_index
                  << " sample " << sample_index
                  << " differs from legacy target evolution: pos=" << pos_err
                  << ", rot=" << rot_err;
          fail(message.str());
        }
      }
    }
  }

  if (attached_sample_count == 0) {
    fail("fixture has no attached state samples");
  }
  if (
    uses_committed_fixture &&
    (!saw_direct || !saw_regrasp || attached_sample_count != 150)) {
    fail("committed fixture coverage changed unexpectedly");
  }
  if (max_position_error > 2e-10 || max_rotation_error > 2e-10) {
    fail("legacy/core error summary exceeds tolerance");
  }
  std::cout << "Compared " << attached_sample_count << " attached samples from "
            << fixture.source << ": max position error=" << max_position_error
            << ", max rotation error=" << max_rotation_error << '\n';
}
