#include <stacking_core/planner/excavator/closed_form_initializer.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace stacking_core {
  namespace {

    constexpr Scalar joint_margin = 0.01;

    struct excavator_geometry_t {
      Scalar D2 = 0.0;
      Scalar D3 = 0.0;
      Scalar boom_height = 0.0;
      Scalar boom_length = 0.0;
      Scalar arm_length = 0.0;
      Vector3 link_vec = Vector3::Zero();
      Vector3 tilt_to_rotate = Vector3::Zero();
    };

    Matrix3 rot_x(Scalar angle) {
      return Eigen::AngleAxis<Scalar> {angle, Vector3::UnitX()}
        .toRotationMatrix();
    }

    Matrix3 rot_y(Scalar angle) {
      return Eigen::AngleAxis<Scalar> {angle, Vector3::UnitY()}
        .toRotationMatrix();
    }

    Matrix3 rot_z(Scalar angle) {
      return Eigen::AngleAxis<Scalar> {angle, Vector3::UnitZ()}
        .toRotationMatrix();
    }

    Quaternion rpy(Scalar roll, Scalar pitch, Scalar yaw) {
      return Quaternion {
        Eigen::AngleAxis<Scalar> {yaw, Vector3::UnitZ()} *
        Eigen::AngleAxis<Scalar> {pitch, Vector3::UnitY()} *
        Eigen::AngleAxis<Scalar> {roll, Vector3::UnitX()}};
    }

    Eigen::Matrix4d dh_matrix(Scalar theta, Scalar d, Scalar a, Scalar alpha) {
      Scalar const c = std::cos(theta);
      Scalar const s = std::sin(theta);
      Scalar const ca = std::cos(alpha);
      Scalar const sa = std::sin(alpha);
      Eigen::Matrix4d T;
      T << c, -s * ca, s * sa, a * c, s, c * ca, -c * sa, a * s, 0.0, sa, ca, d,
        0.0, 0.0, 0.0, 1.0;
      return T;
    }

    excavator_geometry_t hard_coded_geometry() {
      Scalar const pi = std::numbers::pi;
      std::array<std::array<Scalar, 4>, 9> const dh {{
        {0.0, 1.0670, 0.0, pi / 2.0},
        {pi / 2.0, 0.06, 0.0, pi / 2.0},
        {pi / 2.0, 0.117, 0.0, pi / 2.0},
        {pi / 2.0, 0.738, 0.0, pi / 2.0},
        {0.0, 0.0, 5.7, 0.0},
        {0.0, 0.0, 2.9107292, 0.0},
        {0.0, 0.0, 0.55329541, -pi / 2.0},
        {pi / 2.0, 0.47, 0.0, pi / 2.0 - 0.073663660},
        {-pi / 2.0, 0.0, 0.0, 0.0},
      }};

      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      std::array<Vector3, 10> points;
      points[0] = T.topRightCorner<3, 1>();
      for (std::size_t i = 0; i < dh.size(); ++i) {
        T *= dh_matrix(dh[i][0], dh[i][1], dh[i][2], dh[i][3]);
        points[i + 1] = T.topRightCorner<3, 1>();
      }

      Vector3 const& boom = points[4];
      Vector3 const& arm = points[5];
      Vector3 const& link = points[6];
      Vector3 const& tilt = points[7];
      Vector3 const& rotate = points[8];
      return excavator_geometry_t {
        .D2 = -boom.y(),
        .D3 = boom.x(),
        .boom_height = boom.z(),
        .boom_length = (arm - boom).norm(),
        .arm_length = (link - arm).norm(),
        .link_vec = tilt - link,
        .tilt_to_rotate = rotate - tilt,
      };
    }

    std::optional<std::array<Scalar, 6>> closed_form_seed(
      pose_t const& root_from_end) {
      static excavator_geometry_t const geometry = hard_coded_geometry();
      Vector3 const& target = root_from_end.position;
      Matrix3 const R_target = root_from_end.orientation.toRotationMatrix() *
        rot_x(std::numbers::pi) * rot_y(-std::numbers::pi / 2.0);

      Scalar const ground_radius = std::hypot(target.x(), target.y());
      Scalar const phi = std::atan2(target.y(), target.x());
      Scalar const offset_ratio = std::clamp(
        -geometry.D2 / (ground_radius + 1e-9), Scalar {-1.0}, Scalar {1.0});
      Scalar const q_swing = phi - std::asin(offset_ratio);

      Matrix3 const R_swing = rot_z(q_swing);
      Matrix3 const R_swing_inv = R_swing.transpose();
      Vector3 const pivot_local {
        geometry.D3, -geometry.D2, geometry.boom_height};
      Vector3 const pivot_root = R_swing * pivot_local;
      Vector3 const target_planar = R_swing_inv * (target - pivot_root);
      Matrix3 const R_local = R_swing_inv * R_target;

      Scalar const q_tilt =
        std::asin(std::clamp(R_local(1, 0), Scalar {-1.0}, Scalar {1.0}));
      Scalar const q_rotate = std::atan2(-R_local(1, 2), R_local(1, 1));
      Matrix3 const R_pitch = R_local * rot_x(-q_rotate) * rot_z(-q_tilt);
      Scalar const pitch_total = -std::atan2(R_pitch(0, 2), R_pitch(0, 0));

      Vector3 const tilted_rotate = rot_z(q_tilt) * geometry.tilt_to_rotate;
      Vector3 const link_to_rotate = geometry.link_vec + tilted_rotate;
      Vector3 const backstep = rot_y(-pitch_total) * link_to_rotate;
      Vector3 const link_target = target_planar - backstep;

      Scalar const radius = link_target.x();
      Scalar const height = link_target.z();
      Scalar const distance_sq = radius * radius + height * height;
      Scalar const distance = std::sqrt(distance_sq);
      if (
        !std::isfinite(distance) ||
        distance > geometry.boom_length + geometry.arm_length + 1e-4 ||
        distance <
          std::abs(geometry.boom_length - geometry.arm_length) - 1e-4 ||
        distance <= 0.0) {
        return std::nullopt;
      }

      Scalar const cos_arm =
        (distance_sq - geometry.boom_length * geometry.boom_length -
         geometry.arm_length * geometry.arm_length) /
        (2.0 * geometry.boom_length * geometry.arm_length);
      Scalar const q_arm =
        -std::acos(std::clamp(cos_arm, Scalar {-1.0}, Scalar {1.0}));
      Scalar const alpha = std::atan2(height, radius);
      Scalar const beta = std::acos(std::clamp(
        (geometry.boom_length * geometry.boom_length + distance_sq -
         geometry.arm_length * geometry.arm_length) /
          (2.0 * geometry.boom_length * distance),
        Scalar {-1.0}, Scalar {1.0}));
      Scalar const q_boom = alpha + beta;
      Scalar const q_bucket = pitch_total - q_boom - q_arm;

      std::array<Scalar, 6> const result {q_swing,  q_boom, q_arm,
                                          q_bucket, q_tilt, q_rotate};
      for (Scalar value : result) {
        if (!std::isfinite(value)) {
          return std::nullopt;
        }
      }
      return result;
    }

    Scalar wrap_to_pi(Scalar angle) {
      angle = std::fmod(angle + std::numbers::pi, 2.0 * std::numbers::pi);
      if (angle < 0.0) {
        angle += 2.0 * std::numbers::pi;
      }
      return angle - std::numbers::pi;
    }

    LinkId root_of(KinematicModel const& model, LinkId link) {
      while (kinematic_joint_t const* joint = model.parentJoint(link)) {
        link = joint->parent;
      }
      return link;
    }

    std::array<JointId, 6> joints(excavator_ik_chain_t const& chain) {
      return {chain.swing,  chain.boom, chain.arm,
              chain.bucket, chain.tilt, chain.rotate};
    }

    bool matches_hard_coded_geometry(
      std::shared_ptr<KinematicModel const> const& model,
      std::array<JointId, 6> const& chain_joints) {
      std::array<Vector3, 6> const expected_axes {
        Vector3::UnitZ(),  -Vector3::UnitY(), -Vector3::UnitY(),
        -Vector3::UnitY(), Vector3::UnitZ(),  Vector3::UnitZ()};
      std::array<Vector3, 6> const expected_origins {
        Vector3 {0.0, 0.0, 1.0670},   Vector3 {0.117, -0.06, 0.738},
        Vector3 {2.718, 0.0, 5.0103}, Vector3 {-0.2142, 0.0, -2.9029},
        Vector3 {0.0, 0.0, -0.5548},  Vector3 {0.0, 0.0, -0.47},
      };
      std::array<Quaternion, 6> const expected_orientations {
        rpy(0.0, 0.0, 0.0), rpy(0.0, 1.07374, 0.0), rpy(0.0, -2.7182, 0.0),
        rpy(0.0, 0.0, 0.0), rpy(0.0, -1.5708, 0.0), rpy(3.1415, -1.5708, 0.0),
      };
      constexpr Scalar tol = 2e-3;
      for (std::size_t i = 0; i < chain_joints.size(); ++i) {
        kinematic_joint_t const& joint = model->joint(chain_joints[i]);
        if (
          !joint.axis.isApprox(expected_axes[i], 1e-8) ||
          (joint.parent_from_child_zero.position - expected_origins[i])
              .norm() >= tol ||
          joint.parent_from_child_zero.orientation.angularDistance(
            expected_orientations[i]) >= 1e-8) {
          return false;
        }
      }
      return true;
    }

  }  // namespace

  ExcavatorIkInitializer::ExcavatorIkInitializer(
    std::shared_ptr<KinematicModel const> model, excavator_ik_chain_t chain)
      : model_(std::move(model)), chain_(std::move(chain)) {
    if (model_ == nullptr) {
      throw std::invalid_argument("excavator IK model must not be null");
    }
    if (!is_valid(chain_.end_from_task)) {
      throw std::invalid_argument(
        "excavator IK end-from-task pose must be valid");
    }
    if (model_->findLink(chain_.end_link) == nullptr) {
      throw std::invalid_argument(
        "excavator IK end link must belong to the model");
    }

    std::array<JointId, 6> const chain_joints = joints(chain_);
    std::unordered_set<std::size_t> dofs;
    for (std::size_t i = 0; i < chain_joints.size(); ++i) {
      kinematic_joint_t const* joint = model_->findJoint(chain_joints[i]);
      if (
        joint == nullptr || joint->type != joint_type_e::revolute ||
        joint->mimic.has_value() || !joint->limit.has_value()) {
        throw std::invalid_argument(
          "excavator IK chain requires six limited independent revolute "
          "joints");
      }
      std::optional<std::size_t> const dof =
        model_->degreeOfFreedomIndex(joint->id);
      if (!dof.has_value() || !dofs.emplace(*dof).second) {
        throw std::invalid_argument(
          "excavator IK joints must map to six unique coordinates");
      }
      dof_indices_[i] = *dof;
    }

    for (std::size_t i = 1; i < chain_joints.size(); ++i) {
      kinematic_joint_t const& parent = model_->joint(chain_joints[i - 1]);
      kinematic_joint_t const& child = model_->joint(chain_joints[i]);
      if (parent.child != child.parent) {
        throw std::invalid_argument(
          "excavator IK joints must form one direct serial chain");
      }
    }
    if (model_->joint(chain_.rotate).child != chain_.end_link) {
      throw std::invalid_argument(
        "excavator IK rotate joint must terminate at the end link");
    }
    if (!matches_hard_coded_geometry(model_, chain_joints)) {
      throw std::invalid_argument(
        "excavator IK chain does not match the hard-coded VDK23_CX geometry");
    }
  }

  excavator_ik_seed_result_t ExcavatorIkInitializer::seed(
    KinematicState const& state, pose_t const& frame_from_task) const {
    if (state.modelPtr() != model_) {
      return excavator_ik_seed_result_t {
        .status = solve_status_e::invalid_problem,
        .positions = state.positions(),
        .failure =
          {
            .code = "model_mismatch",
            .message = "excavator IK state uses a different kinematic model",
            .retryable = false,
          },
      };
    }
    if (!is_valid(frame_from_task)) {
      return excavator_ik_seed_result_t {
        .status = solve_status_e::invalid_problem,
        .positions = state.positions(),
        .failure =
          {
            .code = "invalid_target",
            .message = "excavator IK target pose must be valid",
            .retryable = false,
          },
      };
    }

    LinkId const root = root_of(*model_, chain_.end_link);
    pose_t const root_from_task =
      compose(inverse(state.frameFromRoot(root)), frame_from_task);
    pose_t const root_from_end =
      compose(root_from_task, inverse(chain_.end_from_task));
    std::optional<std::array<Scalar, 6>> values =
      closed_form_seed(root_from_end);
    if (!values.has_value()) {
      return excavator_ik_seed_result_t {
        .status = solve_status_e::infeasible,
        .positions = state.positions(),
        .failure =
          {
            .code = "closed_form_infeasible",
            .message =
              "excavator closed-form initializer found no elbow-down seed",
            .retryable = true,
          },
      };
    }

    for (Scalar& value : *values) {
      value = wrap_to_pi(value);
    }
    (*values)[4] = -(*values)[4];
    (*values)[5] = -(*values)[5];

    Eigen::VectorXd positions = state.positions();
    std::array<JointId, 6> const chain_joints = joints(chain_);
    for (std::size_t i = 0; i < values->size(); ++i) {
      joint_limit_t const& limit = *model_->joint(chain_joints[i]).limit;
      Scalar const lower = limit.lower + joint_margin;
      Scalar const upper = limit.upper - joint_margin;
      positions(static_cast<Eigen::Index>(dof_indices_[i])) =
        std::clamp((*values)[i], lower, upper);
    }
    return excavator_ik_seed_result_t {
      .status = solve_status_e::success,
      .positions = std::move(positions),
      .failure = {},
    };
  }

}  // namespace stacking_core
