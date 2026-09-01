#include <stacking_core/kinematics/model.hpp>

#include <stdexcept>
#include <utility>
#include <vector>

namespace {

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("kinematic model test requirement failed");
  }
}

stacking_core::kinematic_model_config_t model_config() {
  using namespace stacking_core;
  return kinematic_model_config_t {
    .links = {
      kinematic_link_t {.id = LinkId {1}, .name = "base"},
      kinematic_link_t {.id = LinkId {2}, .name = "arm"},
      kinematic_link_t {.id = LinkId {3}, .name = "slider"},
      kinematic_link_t {.id = LinkId {4}, .name = "finger"},
    },
    .joints = {
      kinematic_joint_t {
        .id = JointId {10},
        .name = "shoulder",
        .type = joint_type_e::revolute,
        .parent = LinkId {1},
        .child = LinkId {2},
        .parent_from_child_zero = pose_t {},
        .axis = Vector3::UnitZ(),
        .limit = joint_limit_t {.lower = -1.0, .upper = 1.0},
        .mimic = std::nullopt,
      },
      kinematic_joint_t {
        .id = JointId {11},
        .name = "extension",
        .type = joint_type_e::prismatic,
        .parent = LinkId {2},
        .child = LinkId {3},
        .parent_from_child_zero = pose_t {},
        .axis = 2.0 * Vector3::UnitX(),
        .limit = joint_limit_t {.lower = 0.0, .upper = 2.0},
        .mimic = std::nullopt,
      },
      kinematic_joint_t {
        .id = JointId {12},
        .name = "finger_mimic",
        .type = joint_type_e::revolute,
        .parent = LinkId {2},
        .child = LinkId {4},
        .parent_from_child_zero = pose_t {},
        .axis = Vector3::UnitZ(),
        .limit = joint_limit_t {.lower = -1.0, .upper = 1.0},
        .mimic = joint_mimic_t {
          .source = JointId {10},
          .multiplier = -1.0,
          .offset = 0.1,
        },
      },
    },
  };
}

}  // namespace

int main() {
  using namespace stacking_core;

  KinematicModel model {model_config()};
  require(model.linkCount() == 4);
  require(model.jointCount() == 3);
  require(model.degreeOfFreedomCount() == 2);
  require(model.roots().size() == 1);
  require(model.roots()[0] == LinkId {1});
  require(model.findLink("arm")->id == LinkId {2});
  require(model.findJoint("extension")->id == JointId {11});
  require(model.parentJoint(LinkId {1}) == nullptr);
  require(model.parentJoint(LinkId {2})->id == JointId {10});
  require(model.childJoints(LinkId {2}).size() == 2);
  require(model.degreeOfFreedomJoint(0) == JointId {10});
  require(model.degreeOfFreedomJoint(1) == JointId {11});
  require(model.degreeOfFreedomIndex(JointId {12}) == 0);
  require(model.joint(JointId {11}).axis.isApprox(Vector3::UnitX()));

  Eigen::Vector2d positions {0.5, 0.25};
  require(model.jointPosition(JointId {10}, positions) == 0.5);
  require(model.jointPosition(JointId {12}, positions) == -0.4);
  require(model.positionsWithinLimits(positions));
  positions[0] = 2.0;
  require(!model.positionsWithinLimits(positions));

  bool rejected_cycle = false;
  try {
    KinematicModel const invalid_model {kinematic_model_config_t {
      .links = {
        kinematic_link_t {.id = LinkId {20}, .name = "a"},
        kinematic_link_t {.id = LinkId {21}, .name = "b"},
      },
      .joints = {
        kinematic_joint_t {
          .id = JointId {20},
          .name = "a_to_b",
          .type = joint_type_e::fixed,
          .parent = LinkId {20},
          .child = LinkId {21},
          .parent_from_child_zero = pose_t {},
          .axis = Vector3::UnitX(),
          .limit = std::nullopt,
          .mimic = std::nullopt,
        },
        kinematic_joint_t {
          .id = JointId {21},
          .name = "b_to_a",
          .type = joint_type_e::fixed,
          .parent = LinkId {21},
          .child = LinkId {20},
          .parent_from_child_zero = pose_t {},
          .axis = Vector3::UnitX(),
          .limit = std::nullopt,
          .mimic = std::nullopt,
        },
      },
    }};
    (void)invalid_model;
  } catch (std::invalid_argument const&) {
    rejected_cycle = true;
  }
  require(rejected_cycle);

  bool rejected_mimic = false;
  try {
    kinematic_model_config_t config = model_config();
    config.joints[2].mimic->source = JointId {999};
    KinematicModel const invalid_model {std::move(config)};
    (void)invalid_model;
  } catch (std::invalid_argument const&) {
    rejected_mimic = true;
  }
  require(rejected_mimic);
}
