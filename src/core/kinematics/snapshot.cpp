#include <stacking_core/kinematics/snapshot.hpp>

#include <Eigen/Geometry>

#include <stdexcept>
#include <utility>

namespace stacking_core {
namespace {

pose_t joint_motion(kinematic_joint_t const& joint, Scalar position) {
  switch (joint.type) {
  case joint_type_e::fixed:
    return pose_t {};
  case joint_type_e::revolute:
    return pose_t {
      Vector3::Zero(),
      Quaternion {Eigen::AngleAxis<Scalar> {position, joint.axis}},
    };
  case joint_type_e::prismatic:
    return pose_t {position * joint.axis, Quaternion::Identity()};
  }
  throw std::logic_error("unknown kinematic joint type");
}

}  // namespace

KinematicSnapshot::KinematicSnapshot(
  FrameId frame,
  std::shared_ptr<KinematicModel const> model,
  std::vector<pose_t> frame_from_links,
  std::vector<Matrix6X> link_jacobians)
    : frame_(frame),
      model_(std::move(model)),
      frame_from_links_(std::move(frame_from_links)),
      link_jacobians_(std::move(link_jacobians)) {
}

pose_t const& KinematicSnapshot::frameFromLink(std::size_t index) const {
  return frame_from_links_.at(index);
}

pose_t const& KinematicSnapshot::frameFromLink(LinkId link) const {
  return frame_from_links_.at(model_->linkIndex(link));
}

Matrix6X const& KinematicSnapshot::linkJacobian(std::size_t index) const {
  return link_jacobians_.at(index);
}

Matrix6X const& KinematicSnapshot::linkJacobian(LinkId link) const {
  return link_jacobians_.at(model_->linkIndex(link));
}

KinematicSnapshot forward_kinematics(KinematicState const& state) {
  KinematicModel const& model = state.model();
  std::vector<pose_t> frame_from_links(model.linkCount());
  std::vector<Matrix6X> link_jacobians(
    model.linkCount(),
    Matrix6X::Zero(
      6, static_cast<Eigen::Index>(model.degreeOfFreedomCount())));

  std::size_t root_index = 0;
  for (std::size_t link_index : model.traversal_order_) {
    std::optional<std::size_t> const parent_joint =
      model.parent_joints_[link_index];
    if (!parent_joint.has_value()) {
      LinkId const root = model.links_[link_index].id;
      while (model.roots_[root_index] != root) {
        ++root_index;
      }
      frame_from_links[link_index] = state.frameFromRoot(root);
      ++root_index;
      continue;
    }

    kinematic_joint_t const& joint = model.joints_[*parent_joint];
    std::size_t const parent_link_index = model.linkIndex(joint.parent);
    pose_t const parent_from_child = compose(
      joint.parent_from_child_zero,
      joint_motion(joint, state.position(joint.id)));
    frame_from_links[link_index] = compose(
      frame_from_links[parent_link_index], parent_from_child);

    Matrix6X& jacobian = link_jacobians[link_index];
    Matrix6X const& parent_jacobian = link_jacobians[parent_link_index];
    Vector3 const parent_to_child =
      frame_from_links[link_index].position -
      frame_from_links[parent_link_index].position;
    jacobian.bottomRows<3>() = parent_jacobian.bottomRows<3>();
    jacobian.topRows<3>() = parent_jacobian.topRows<3>() -
      skew(parent_to_child) * parent_jacobian.bottomRows<3>();

    std::optional<std::size_t> const dof_index =
      model.dof_indices_[*parent_joint];
    if (!dof_index.has_value()) {
      continue;
    }
    Scalar const multiplier = model.position_multipliers_[*parent_joint];
    Vector3 const axis_in_frame = transform_vector(
      frame_from_links[parent_link_index],
      transform_vector(joint.parent_from_child_zero, joint.axis));
    Eigen::Index const column = static_cast<Eigen::Index>(*dof_index);
    if (joint.type == joint_type_e::revolute) {
      jacobian.bottomRows<3>().col(column) += multiplier * axis_in_frame;
    } else if (joint.type == joint_type_e::prismatic) {
      jacobian.topRows<3>().col(column) += multiplier * axis_in_frame;
    }
  }

  return KinematicSnapshot {
    state.frame(),
    state.modelPtr(),
    std::move(frame_from_links),
    std::move(link_jacobians)};
}

}  // namespace stacking_core
