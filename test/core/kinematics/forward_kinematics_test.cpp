#include <stacking_core/kinematics.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <stdexcept>

namespace {

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("forward kinematics test requirement failed");
  }
}

std::shared_ptr<stacking_core::KinematicModel const> make_model() {
  using namespace stacking_core;
  return std::make_shared<KinematicModel>(kinematic_model_config_t {
    .links = {
      kinematic_link_t {.id = LinkId {1}, .name = "base"},
      kinematic_link_t {.id = LinkId {2}, .name = "arm"},
      kinematic_link_t {.id = LinkId {3}, .name = "tool"},
      kinematic_link_t {.id = LinkId {4}, .name = "finger"},
    },
    .joints = {
      kinematic_joint_t {
        .id = JointId {1},
        .name = "shoulder",
        .type = joint_type_e::revolute,
        .parent = LinkId {1},
        .child = LinkId {2},
        .parent_from_child_zero = pose_t {
          Vector3 {1.0, 0.0, 0.0}, Quaternion::Identity()},
        .axis = Vector3::UnitZ(),
        .limit = std::nullopt,
        .mimic = std::nullopt,
      },
      kinematic_joint_t {
        .id = JointId {2},
        .name = "extension",
        .type = joint_type_e::prismatic,
        .parent = LinkId {2},
        .child = LinkId {3},
        .parent_from_child_zero = pose_t {
          Vector3 {1.0, 0.0, 0.0}, Quaternion::Identity()},
        .axis = Vector3::UnitX(),
        .limit = joint_limit_t {.lower = 0.0, .upper = 1.0},
        .mimic = std::nullopt,
      },
      kinematic_joint_t {
        .id = JointId {3},
        .name = "finger_mimic",
        .type = joint_type_e::revolute,
        .parent = LinkId {2},
        .child = LinkId {4},
        .parent_from_child_zero = pose_t {
          Vector3 {0.0, 1.0, 0.0}, Quaternion::Identity()},
        .axis = Vector3::UnitZ(),
        .limit = std::nullopt,
        .mimic = joint_mimic_t {
          .source = JointId {1},
          .multiplier = -1.0,
          .offset = 0.1,
        },
      },
    },
  });
}

}  // namespace

int main() {
  using namespace stacking_core;

  std::shared_ptr<KinematicModel const> const model = make_model();
  KinematicState state {FrameId {8}, model};
  state.setFrameFromRoot(
    LinkId {1}, pose_t {Vector3 {10.0, 0.0, 0.0}, Quaternion::Identity()});
  state.setPosition(JointId {1}, 0.5);
  state.setPosition(JointId {2}, 0.25);

  require(state.frame() == FrameId {8});
  require(state.position(JointId {3}) == -0.4);
  require(state.positionsWithinLimits());

  KinematicSnapshot const snapshot = forward_kinematics(state);
  require(snapshot.frame() == FrameId {8});
  require(snapshot.modelPtr() == model);
  require(snapshot.frameFromLink(LinkId {1}).position.isApprox(
    Vector3 {10.0, 0.0, 0.0}));
  require(snapshot.frameFromLink(LinkId {2}).position.isApprox(
    Vector3 {11.0, 0.0, 0.0}));

  Quaternion const arm_rotation {
    Eigen::AngleAxis<Scalar> {0.5, Vector3::UnitZ()}};
  Vector3 const expected_tool_position =
    Vector3 {11.0, 0.0, 0.0} + arm_rotation * Vector3 {1.25, 0.0, 0.0};
  require(snapshot.frameFromLink(LinkId {3}).position.isApprox(
    expected_tool_position));
  require(snapshot.frameFromLink(LinkId {3}).orientation.isApprox(arm_rotation));

  Matrix6X expected_tool_jacobian = Matrix6X::Zero(6, 2);
  Vector3 const arm_to_tool = arm_rotation * Vector3 {1.25, 0.0, 0.0};
  expected_tool_jacobian.topRows<3>().col(0) =
    Vector3::UnitZ().cross(arm_to_tool);
  expected_tool_jacobian.bottomRows<3>().col(0) = Vector3::UnitZ();
  expected_tool_jacobian.topRows<3>().col(1) =
    arm_rotation * Vector3::UnitX();
  require(snapshot.linkJacobian(LinkId {3}).isApprox(expected_tool_jacobian));

  Quaternion const expected_finger_rotation = arm_rotation * Quaternion {
    Eigen::AngleAxis<Scalar> {-0.4, Vector3::UnitZ()}};
  require(snapshot.frameFromLink(LinkId {4}).orientation.isApprox(
    expected_finger_rotation));
  Vector3 const arm_to_finger = arm_rotation * Vector3 {0.0, 1.0, 0.0};
  require(snapshot.linkJacobian(LinkId {4}).topRows<3>().col(0).isApprox(
    Vector3::UnitZ().cross(arm_to_finger)));
  require(snapshot.linkJacobian(LinkId {4}).bottomRows<3>().col(0).isZero());

  bool rejected_mimic_set = false;
  try {
    state.setPosition(JointId {3}, 0.0);
  } catch (std::invalid_argument const&) {
    rejected_mimic_set = true;
  }
  require(rejected_mimic_set);

  state.setPosition(JointId {2}, 2.0);
  require(!state.positionsWithinLimits());
}
