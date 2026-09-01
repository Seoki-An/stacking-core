#pragma once

#include <stacking_core/transform.hpp>

#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace stacking_core {

class KinematicSnapshot;
class KinematicState;

enum class joint_type_e {
  fixed,
  revolute,
  prismatic,
};

struct joint_limit_t {
  Scalar lower = 0.0;
  Scalar upper = 0.0;
};

struct joint_mimic_t {
  JointId source;
  Scalar multiplier = 1.0;
  Scalar offset = 0.0;
};

struct kinematic_link_t {
  LinkId id;
  std::string name;
};

struct kinematic_joint_t {
  JointId id;
  std::string name;
  joint_type_e type = joint_type_e::fixed;
  LinkId parent;
  LinkId child;

  // Transform from the parent link to the child link at zero joint position.
  pose_t parent_from_child_zero;

  // Expressed in the child/joint frame at zero position.
  Vector3 axis = Vector3::UnitX();
  std::optional<joint_limit_t> limit;
  std::optional<joint_mimic_t> mimic;
};

struct kinematic_model_config_t {
  std::vector<kinematic_link_t> links;
  std::vector<kinematic_joint_t> joints;
};

// Immutable kinematic topology. It intentionally contains no URDF parser,
// body instances, world poses, or solver state.
class KinematicModel {
public:
  explicit KinematicModel(kinematic_model_config_t config);

  KinematicModel(KinematicModel const&) = delete;
  KinematicModel& operator=(KinematicModel const&) = delete;
  KinematicModel(KinematicModel&&) = delete;
  KinematicModel& operator=(KinematicModel&&) = delete;

  [[nodiscard]] std::size_t linkCount() const noexcept {
    return links_.size();
  }

  [[nodiscard]] std::size_t jointCount() const noexcept {
    return joints_.size();
  }

  [[nodiscard]] std::size_t degreeOfFreedomCount() const noexcept {
    return dof_joints_.size();
  }

  [[nodiscard]] kinematic_link_t const& link(std::size_t index) const;
  [[nodiscard]] kinematic_link_t const& link(LinkId id) const;
  [[nodiscard]] kinematic_link_t const* findLink(LinkId id) const noexcept;
  [[nodiscard]] kinematic_link_t const* findLink(std::string_view name) const noexcept;

  [[nodiscard]] kinematic_joint_t const& joint(std::size_t index) const;
  [[nodiscard]] kinematic_joint_t const& joint(JointId id) const;
  [[nodiscard]] kinematic_joint_t const* findJoint(JointId id) const noexcept;
  [[nodiscard]] kinematic_joint_t const* findJoint(std::string_view name) const noexcept;

  [[nodiscard]] std::span<LinkId const> roots() const noexcept {
    return roots_;
  }

  [[nodiscard]] std::span<JointId const> childJoints(LinkId link) const;
  [[nodiscard]] kinematic_joint_t const* parentJoint(LinkId link) const;

  // Independent generalized coordinate ordering. Fixed and mimic joints do
  // not receive their own coordinate.
  [[nodiscard]] JointId degreeOfFreedomJoint(std::size_t index) const;
  [[nodiscard]] std::optional<std::size_t> degreeOfFreedomIndex(
    JointId joint) const;

  [[nodiscard]] Scalar jointPosition(
    JointId joint, Eigen::Ref<Eigen::VectorXd const> positions) const;
  [[nodiscard]] bool positionsWithinLimits(
    Eigen::Ref<Eigen::VectorXd const> positions,
    Scalar tol = 0.0) const;

private:
  friend class KinematicSnapshot;
  friend KinematicSnapshot forward_kinematics(KinematicState const& state);

  [[nodiscard]] std::size_t linkIndex(LinkId id) const;
  [[nodiscard]] std::size_t jointIndex(JointId id) const;

  std::vector<kinematic_link_t> links_;
  std::vector<kinematic_joint_t> joints_;
  std::vector<LinkId> roots_;
  std::vector<std::vector<JointId>> child_joints_;
  std::vector<std::optional<std::size_t>> parent_joints_;
  std::vector<JointId> dof_joints_;
  std::vector<std::optional<std::size_t>> dof_indices_;
  std::vector<Scalar> position_multipliers_;
  std::vector<Scalar> position_offsets_;
  std::vector<std::size_t> traversal_order_;
};

}  // namespace stacking_core
