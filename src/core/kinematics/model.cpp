#include <stacking_core/kinematics/model.hpp>

#include <cmath>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace stacking_core {
namespace {

bool is_movable(joint_type_e type) noexcept {
  return type == joint_type_e::revolute || type == joint_type_e::prismatic;
}

}  // namespace

KinematicModel::KinematicModel(kinematic_model_config_t config)
    : links_(std::move(config.links)), joints_(std::move(config.joints)) {
  if (links_.empty()) {
    throw std::invalid_argument("kinematic model requires at least one link");
  }

  std::unordered_map<LinkId, std::size_t> link_indices;
  std::unordered_set<std::string> link_names;
  for (std::size_t index = 0; index < links_.size(); ++index) {
    kinematic_link_t const& link = links_[index];
    if (!link.id.valid()) {
      throw std::invalid_argument("kinematic link ID must be valid");
    }
    if (!link_indices.emplace(link.id, index).second) {
      throw std::invalid_argument("kinematic link IDs must be unique");
    }
    if (link.name.empty()) {
      throw std::invalid_argument("kinematic link name must not be empty");
    }
    if (!link_names.emplace(link.name).second) {
      throw std::invalid_argument("kinematic link names must be unique");
    }
  }

  child_joints_.resize(links_.size());
  parent_joints_.resize(links_.size());
  dof_indices_.resize(joints_.size());
  position_multipliers_.assign(joints_.size(), 0.0);
  position_offsets_.assign(joints_.size(), 0.0);

  std::unordered_map<JointId, std::size_t> joint_indices;
  std::unordered_set<std::string> joint_names;
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    kinematic_joint_t& joint = joints_[index];
    if (!joint.id.valid()) {
      throw std::invalid_argument("kinematic joint ID must be valid");
    }
    if (!joint_indices.emplace(joint.id, index).second) {
      throw std::invalid_argument("kinematic joint IDs must be unique");
    }
    if (joint.name.empty()) {
      throw std::invalid_argument("kinematic joint name must not be empty");
    }
    if (!joint_names.emplace(joint.name).second) {
      throw std::invalid_argument("kinematic joint names must be unique");
    }
    auto const parent = link_indices.find(joint.parent);
    auto const child = link_indices.find(joint.child);
    if (parent == link_indices.end() || child == link_indices.end()) {
      throw std::invalid_argument("kinematic joint links must belong to the model");
    }
    if (joint.parent == joint.child) {
      throw std::invalid_argument("kinematic joint cannot connect a link to itself");
    }
    if (!is_valid(joint.parent_from_child_zero)) {
      throw std::invalid_argument("kinematic joint zero pose must be valid");
    }
    if (parent_joints_[child->second].has_value()) {
      throw std::invalid_argument("kinematic link cannot have multiple parent joints");
    }
    parent_joints_[child->second] = index;
    child_joints_[parent->second].push_back(joint.id);

    if (is_movable(joint.type)) {
      Scalar const axis_norm = joint.axis.norm();
      if (!joint.axis.allFinite() || !std::isfinite(axis_norm) ||
          axis_norm <= 0.0) {
        throw std::invalid_argument("movable joint axis must be finite and non-zero");
      }
      joint.axis /= axis_norm;
    } else if (joint.limit.has_value() || joint.mimic.has_value()) {
      throw std::invalid_argument("fixed joint cannot have a limit or mimic source");
    }

    if (joint.limit.has_value()) {
      joint_limit_t const& limit = *joint.limit;
      if (!std::isfinite(limit.lower) || !std::isfinite(limit.upper) ||
          limit.lower > limit.upper) {
        throw std::invalid_argument("joint limit must be finite and ordered");
      }
    }
    if (joint.mimic.has_value()) {
      joint_mimic_t const& mimic = *joint.mimic;
      if (!mimic.source.valid() || !std::isfinite(mimic.multiplier) ||
          !std::isfinite(mimic.offset)) {
        throw std::invalid_argument("joint mimic parameters must be valid and finite");
      }
    }
  }

  for (std::size_t index = 0; index < links_.size(); ++index) {
    if (!parent_joints_[index].has_value()) {
      roots_.push_back(links_[index].id);
    }
  }

  std::vector<int> link_marks(links_.size(), 0);
  std::function<void(std::size_t)> visit_link = [&](std::size_t link_index) {
    if (link_marks[link_index] == 1) {
      throw std::invalid_argument("kinematic model must not contain a cycle");
    }
    if (link_marks[link_index] == 2) {
      return;
    }
    link_marks[link_index] = 1;
    traversal_order_.push_back(link_index);
    for (JointId joint_id : child_joints_[link_index]) {
      kinematic_joint_t const& child_joint = joints_[joint_indices.at(joint_id)];
      visit_link(link_indices.at(child_joint.child));
    }
    link_marks[link_index] = 2;
  };
  for (LinkId root : roots_) {
    visit_link(link_indices.at(root));
  }
  if (traversal_order_.size() != links_.size()) {
    throw std::invalid_argument("kinematic model must be an acyclic forest");
  }

  for (std::size_t index = 0; index < joints_.size(); ++index) {
    kinematic_joint_t const& joint = joints_[index];
    if (is_movable(joint.type) && !joint.mimic.has_value()) {
      dof_indices_[index] = dof_joints_.size();
      dof_joints_.push_back(joint.id);
      position_multipliers_[index] = 1.0;
    }
  }

  std::vector<int> joint_marks(joints_.size(), 0);
  std::function<void(std::size_t)> resolve_mimic = [&](std::size_t index) {
    if (!joints_[index].mimic.has_value()) {
      return;
    }
    if (joint_marks[index] == 1) {
      throw std::invalid_argument("kinematic mimic joints must not contain a cycle");
    }
    if (joint_marks[index] == 2) {
      return;
    }
    joint_marks[index] = 1;
    joint_mimic_t const& mimic = *joints_[index].mimic;
    auto const source = joint_indices.find(mimic.source);
    if (source == joint_indices.end()) {
      throw std::invalid_argument("kinematic mimic source must belong to the model");
    }
    if (!is_movable(joints_[source->second].type) ||
        joints_[source->second].type != joints_[index].type) {
      throw std::invalid_argument(
        "kinematic mimic source must be a movable joint of the same type");
    }
    resolve_mimic(source->second);
    dof_indices_[index] = dof_indices_[source->second];
    position_multipliers_[index] =
      mimic.multiplier * position_multipliers_[source->second];
    position_offsets_[index] =
      mimic.multiplier * position_offsets_[source->second] + mimic.offset;
    joint_marks[index] = 2;
  };
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    resolve_mimic(index);
  }
}

kinematic_link_t const& KinematicModel::link(std::size_t index) const {
  return links_.at(index);
}

kinematic_link_t const& KinematicModel::link(LinkId id) const {
  return links_.at(linkIndex(id));
}

kinematic_link_t const* KinematicModel::findLink(LinkId id) const noexcept {
  for (kinematic_link_t const& link_value : links_) {
    if (link_value.id == id) {
      return &link_value;
    }
  }
  return nullptr;
}

kinematic_link_t const* KinematicModel::findLink(std::string_view name) const noexcept {
  for (kinematic_link_t const& link_value : links_) {
    if (link_value.name == name) {
      return &link_value;
    }
  }
  return nullptr;
}

kinematic_joint_t const& KinematicModel::joint(std::size_t index) const {
  return joints_.at(index);
}

kinematic_joint_t const& KinematicModel::joint(JointId id) const {
  return joints_.at(jointIndex(id));
}

kinematic_joint_t const* KinematicModel::findJoint(JointId id) const noexcept {
  for (kinematic_joint_t const& joint_value : joints_) {
    if (joint_value.id == id) {
      return &joint_value;
    }
  }
  return nullptr;
}

kinematic_joint_t const* KinematicModel::findJoint(std::string_view name) const noexcept {
  for (kinematic_joint_t const& joint_value : joints_) {
    if (joint_value.name == name) {
      return &joint_value;
    }
  }
  return nullptr;
}

std::span<JointId const> KinematicModel::childJoints(LinkId link_id) const {
  return child_joints_.at(linkIndex(link_id));
}

kinematic_joint_t const* KinematicModel::parentJoint(LinkId link_id) const {
  std::optional<std::size_t> const index = parent_joints_.at(linkIndex(link_id));
  return index.has_value() ? &joints_[*index] : nullptr;
}

JointId KinematicModel::degreeOfFreedomJoint(std::size_t index) const {
  return dof_joints_.at(index);
}

std::optional<std::size_t> KinematicModel::degreeOfFreedomIndex(
  JointId joint_id) const {
  return dof_indices_.at(jointIndex(joint_id));
}

Scalar KinematicModel::jointPosition(
  JointId joint_id, Eigen::Ref<Eigen::VectorXd const> positions) const {
  if (positions.size() != static_cast<Eigen::Index>(dof_joints_.size())) {
    throw std::invalid_argument("kinematic position vector has the wrong size");
  }
  std::size_t const index = jointIndex(joint_id);
  if (!dof_indices_[index].has_value()) {
    return 0.0;
  }
  return position_multipliers_[index] *
      positions[static_cast<Eigen::Index>(*dof_indices_[index])] +
    position_offsets_[index];
}

bool KinematicModel::positionsWithinLimits(
  Eigen::Ref<Eigen::VectorXd const> positions, Scalar tol) const {
  if (positions.size() != static_cast<Eigen::Index>(dof_joints_.size()) ||
      !positions.allFinite() || !std::isfinite(tol) || tol < 0.0) {
    return false;
  }
  for (kinematic_joint_t const& joint_value : joints_) {
    if (!joint_value.limit.has_value()) {
      continue;
    }
    Scalar const value = jointPosition(joint_value.id, positions);
    if (value < joint_value.limit->lower - tol ||
        value > joint_value.limit->upper + tol) {
      return false;
    }
  }
  return true;
}

std::size_t KinematicModel::linkIndex(LinkId id) const {
  for (std::size_t index = 0; index < links_.size(); ++index) {
    if (links_[index].id == id) {
      return index;
    }
  }
  throw std::out_of_range("kinematic model does not contain the link ID");
}

std::size_t KinematicModel::jointIndex(JointId id) const {
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    if (joints_[index].id == id) {
      return index;
    }
  }
  throw std::out_of_range("kinematic model does not contain the joint ID");
}

}  // namespace stacking_core
