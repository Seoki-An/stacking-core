#include <stacking_core/collision.hpp>
#include <stacking_core/planner/inverse_kinematics.hpp>
#include <stacking_core/planner/motion.hpp>

#include "collision_alm.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace stacking_core {
  namespace {

    using planner_detail::collision_alm_state_t;
    using planner_detail::collision_key_t;

    constexpr int bt_max = 20;
    constexpr Scalar bt_shrink = 0.5;
    constexpr Scalar bt_grow = 1.5;
    constexpr int grasped_collision_boundary = 3;

    enum class planning_mode_e {
      free,
      grasped,
    };

    struct joint_bounds_t {
      Eigen::VectorXd lower;
      Eigen::VectorXd upper;
      std::vector<bool> full_revolution;
      bool valid = true;
    };

    struct path_evaluation_t {
      Scalar cost = 0.0;
      Eigen::VectorXd grad;
      Scalar max_collision_violation = 0.0;
      std::map<collision_key_t, Scalar> collision_violations;
    };

    struct moving_body_t {
      BodyInstance body;
      Matrix6X jac;
    };

    struct collision_constraint_t {
      Scalar violation;
      Eigen::RowVectorXd d_gap;
    };

    struct segment_result_t {
      std::vector<Eigen::VectorXd> path;
      std::vector<std::optional<pose_t>> target_path;
      planner_solver_stats_t solver;
      bool feasible = false;
      planner_failure_t failure;
    };

    struct solver_context_t {
      SceneView const& scene;
      motion_robot_t const& robot;
      motion_planning_config_t const& config;
      planning_mode_e mode;
      std::optional<attachment_t> attachment;
      joint_bounds_t bounds;
    };

    motion_result_t invalid_result(std::string code, std::string message) {
      return motion_result_t {
        .status = solve_status_e::invalid_problem,
        .trajectory = {},
        .solver = {},
        .failure =
          {
            .code = std::move(code),
            .message = std::move(message),
            .retryable = false,
          },
      };
    }

    bool valid_config(motion_planning_config_t const& config) {
      return std::isfinite(config.smooth_weight) &&
        config.smooth_weight > 0.0 && std::isfinite(config.boundary_weight) &&
        config.boundary_weight > 0.0 &&
        std::isfinite(config.collision_weight) &&
        config.collision_weight >= 0.0 &&
        std::isfinite(config.joint_limit_weight) &&
        config.joint_limit_weight >= 0.0 &&
        std::isfinite(config.smooth_boundary_alpha) &&
        std::isfinite(config.swing_smoothness_scale) &&
        config.swing_smoothness_scale > 0.0 &&
        std::isfinite(config.collision_margin) &&
        config.collision_margin >= 0.0 &&
        std::isfinite(config.plane_feasibility_margin) &&
        std::isfinite(config.joint_limit_margin) &&
        config.joint_limit_margin >= 0.0 &&
        std::isfinite(config.grasped_boundary_pos_scale) &&
        config.grasped_boundary_pos_scale > 0.0 &&
        std::isfinite(config.grasped_boundary_rot_scale) &&
        config.grasped_boundary_rot_scale > 0.0 &&
        std::isfinite(config.target_collision_tol) &&
        config.target_collision_tol >= 0.0 && std::isfinite(config.step_size) &&
        std::isfinite(config.collision_penetration_clamp) &&
        config.collision_alm_max_iters > 0 &&
        std::isfinite(config.collision_alm_beta_init) &&
        std::isfinite(config.collision_alm_beta_increase) &&
        config.collision_alm_beta_increase >= 1.0 &&
        std::isfinite(config.collision_alm_tol) && config.collision_alm_tol >= 0.0 &&
        (!config.collision_alm_enabled || config.collision_alm_beta_init > 0.0 ||
         config.collision_weight > 0.0) &&
        config.step_size > 0.0 && config.max_iters > 0 &&
        std::isfinite(config.tol) && config.tol > 0.0 &&
        config.ik_max_iters > 0 && std::isfinite(config.ik_tol) &&
        config.ik_tol > 0.0;
    }

    joint_bounds_t make_joint_bounds(KinematicModel const& model) {
      Eigen::Index const dof =
        static_cast<Eigen::Index>(model.degreeOfFreedomCount());
      joint_bounds_t bounds {
        .lower = Eigen::VectorXd::Constant(
          dof, -std::numeric_limits<Scalar>::infinity()),
        .upper = Eigen::VectorXd::Constant(
          dof, std::numeric_limits<Scalar>::infinity()),
        .full_revolution =
          std::vector<bool>(static_cast<std::size_t>(dof), false),
        .valid = true,
      };
      Eigen::VectorXd const zero = Eigen::VectorXd::Zero(dof);
      for (std::size_t i = 0; i < model.jointCount(); ++i) {
        kinematic_joint_t const& joint = model.joint(i);
        if (!joint.limit.has_value()) {
          continue;
        }
        std::optional<std::size_t> const dof_index =
          model.degreeOfFreedomIndex(joint.id);
        if (!dof_index.has_value()) {
          bounds.valid = false;
          continue;
        }
        Eigen::Index const index = static_cast<Eigen::Index>(*dof_index);
        Scalar const offset = model.jointPosition(joint.id, zero);
        Eigen::VectorXd unit = zero;
        unit(index) = 1.0;
        Scalar const multiplier = model.jointPosition(joint.id, unit) - offset;
        if (std::abs(multiplier) <= std::numeric_limits<Scalar>::epsilon()) {
          bounds.valid = false;
          continue;
        }
        Scalar lower = (joint.limit->lower - offset) / multiplier;
        Scalar upper = (joint.limit->upper - offset) / multiplier;
        if (lower > upper) {
          std::swap(lower, upper);
        }
        bounds.lower(index) = std::max(bounds.lower(index), lower);
        bounds.upper(index) = std::min(bounds.upper(index), upper);
      }
      for (Eigen::Index i = 0; i < dof; ++i) {
        bounds.valid = bounds.valid && bounds.lower(i) <= bounds.upper(i);
        bounds.full_revolution[static_cast<std::size_t>(i)] =
          std::isfinite(bounds.lower(i)) && std::isfinite(bounds.upper(i)) &&
          bounds.upper(i) - bounds.lower(i) >= 2.0 * std::numbers::pi - 1e-9;
      }
      return bounds;
    }

    Eigen::VectorXd clamp(
      Eigen::VectorXd positions, joint_bounds_t const& bounds) {
      return positions.cwiseMin(bounds.upper).cwiseMax(bounds.lower);
    }

    Eigen::VectorXd wrapped_difference(
      Eigen::VectorXd const& to, Eigen::VectorXd const& from,
      joint_bounds_t const& bounds) {
      Eigen::VectorXd diff = to - from;
      for (Eigen::Index i = 0; i < diff.size(); ++i) {
        if (bounds.full_revolution[static_cast<std::size_t>(i)]) {
          diff(i) -= 2.0 * std::numbers::pi *
            std::round(diff(i) / (2.0 * std::numbers::pi));
        }
      }
      return diff;
    }

    Eigen::VectorXd normalize_goal_branch(
      Eigen::VectorXd goal, Eigen::VectorXd const& ref,
      joint_bounds_t const& bounds, bool preserve_branch) {
      if (preserve_branch) {
        return goal;
      }
      for (Eigen::Index i = 0; i < goal.size(); ++i) {
        if (!bounds.full_revolution[static_cast<std::size_t>(i)]) {
          continue;
        }
        Scalar const diff = goal(i) - ref(i);
        Scalar const candidate = ref(i) + diff -
          2.0 * std::numbers::pi * std::round(diff / (2.0 * std::numbers::pi));
        if (candidate >= bounds.lower(i) && candidate <= bounds.upper(i)) {
          goal(i) = candidate;
        }
      }
      return goal;
    }

    Matrix6X body_pose_jacobian(
      KinematicSnapshot const& snapshot, LinkId link,
      pose_t const& link_from_body) {
      Matrix6X const& J_link = snapshot.linkJacobian(link);
      pose_t const& frame_from_link = snapshot.frameFromLink(link);
      pose_t const frame_from_body = compose(frame_from_link, link_from_body);
      Vector3 const offset =
        frame_from_link.orientation * link_from_body.position;
      Matrix6X J = J_link;
      J.topRows<3>() -= skew(offset) * J_link.bottomRows<3>();
      J.bottomRows<3>() =
        frame_from_body.orientation.toRotationMatrix().transpose() *
        J_link.bottomRows<3>();
      return J;
    }

    pose_t tool_pose(
      KinematicSnapshot const& snapshot, motion_robot_t const& robot) {
      return compose(
        snapshot.frameFromLink(robot.tool_link), robot.link_from_tool);
    }

    pose_t interpolation_pose(
      KinematicSnapshot const& snapshot, motion_robot_t const& robot,
      LinkId link) {
      if (link == robot.tool_link) {
        return tool_pose(snapshot, robot);
      }
      return snapshot.frameFromLink(link);
    }

    Matrix6X hybrid_tool_jacobian(
      KinematicSnapshot const& snapshot, motion_robot_t const& robot) {
      return body_pose_jacobian(
        snapshot, robot.tool_link, robot.link_from_tool);
    }

    Vector3 orientation_error(
      Quaternion const& target, Quaternion const& achieved) {
      Matrix3 const R_error =
        (target.conjugate() * achieved).toRotationMatrix();
      Matrix3 const skew_error = R_error - R_error.transpose();
      return 0.5 *
        Vector3 {-skew_error(1, 2), skew_error(0, 2), -skew_error(0, 1)};
    }

    std::optional<Eigen::VectorXd> resolve_goal(
      solver_context_t const& context, Eigen::VectorXd const& initial,
      motion_goal_t const& goal, LinkId& interpolation_link,
      bool& interpolate_tool_frame, planner_failure_t& failure) {
      KinematicModel const& model = context.robot.initial_state.model();
      if (auto const* joint_goal = std::get_if<joint_goal_t>(&goal)) {
        interpolation_link = context.robot.tool_link;
        interpolate_tool_frame = true;
        if (
          joint_goal->positions.size() != initial.size() ||
          !joint_goal->positions.allFinite()) {
          failure = planner_failure_t {
            .code = "invalid_joint_goal",
            .message = "motion joint goal must match the model DoF count",
            .retryable = false,
          };
          return std::nullopt;
        }
        Eigen::VectorXd result = normalize_goal_branch(
          joint_goal->positions, initial, context.bounds,
          joint_goal->preserve_branch);
        if (!model.positionsWithinLimits(result)) {
          failure = planner_failure_t {
            .code = "goal_out_of_joint_limits",
            .message = "motion joint goal violates a joint limit",
            .retryable = false,
          };
          return std::nullopt;
        }
        return result;
      }

      auto const& link_goal = std::get<link_pose_goal_t>(goal);
      interpolation_link = link_goal.link;
      interpolate_tool_frame = false;
      if (
        model.findLink(link_goal.link) == nullptr ||
        !is_valid(link_goal.frame_from_link)) {
        failure = planner_failure_t {
          .code = "invalid_link_goal",
          .message = "motion link goal must reference a valid link and pose",
          .retryable = false,
        };
        return std::nullopt;
      }
      KinematicState state = context.robot.initial_state;
      state.setPositions(initial);
      inverse_kinematics_initialization_e initialization =
        inverse_kinematics_initialization_e::swing;
      if (context.robot.ik_initializer) {
        std::optional<Eigen::VectorXd> const seed =
          context.robot.ik_initializer(
            state, link_goal.link, link_goal.frame_from_link);
        if (
          seed.has_value() && seed->size() == initial.size() &&
          seed->allFinite()) {
          state.setPositions(*seed);
          initialization = inverse_kinematics_initialization_e::provided;
        }
      }
      inverse_kinematics_result_t const ik = solve_inverse_kinematics(
        inverse_kinematics_problem_t {
          .initial_state = state,
          .link = link_goal.link,
          .frame_from_link = link_goal.frame_from_link,
          .position_only = false,
        },
        inverse_kinematics_config_t {
          .max_iters = context.config.ik_max_iters,
          .tol = context.config.ik_tol,
          .initialization = initialization,
        });
      if (ik.status != solve_status_e::success) {
        failure = planner_failure_t {
          .code = "waypoint_ik_failed",
          .message = "motion waypoint could not be resolved by IK",
          .retryable = true,
        };
        return std::nullopt;
      }
      return ik.positions;
    }

    std::vector<Eigen::VectorXd> initialize_path(
      solver_context_t const& context, Eigen::VectorXd const& q_init,
      Eigen::VectorXd const& q_goal, LinkId interpolation_link, int steps,
      bool workspace_init, bool interpolate_tool_frame) {
      std::vector<Eigen::VectorXd> path(static_cast<std::size_t>(steps));
      path.front() = q_init;
      path.back() = q_goal;
      if (!workspace_init) {
        for (int t = 1; t < steps - 1; ++t) {
          Scalar const s = static_cast<Scalar>(t) / (steps - 1);
          path[static_cast<std::size_t>(t)] =
            clamp(q_init + s * (q_goal - q_init), context.bounds);
        }
        return path;
      }

      KinematicState state = context.robot.initial_state;
      state.setPositions(q_init);
      KinematicSnapshot const snapshot_init = forward_kinematics(state);
      pose_t const pose_init = interpolate_tool_frame
        ? interpolation_pose(snapshot_init, context.robot, interpolation_link)
        : snapshot_init.frameFromLink(interpolation_link);
      state.setPositions(q_goal);
      KinematicSnapshot const snapshot_goal = forward_kinematics(state);
      pose_t const pose_goal = interpolate_tool_frame
        ? interpolation_pose(snapshot_goal, context.robot, interpolation_link)
        : snapshot_goal.frameFromLink(interpolation_link);
      Eigen::VectorXd const q_per_step =
        (q_goal - q_init).cwiseAbs() / (steps - 1);
      for (int t = 1; t < steps - 1; ++t) {
        Scalar const s = static_cast<Scalar>(t) / (steps - 1);
        pose_t const target {
          (1.0 - s) * pose_init.position + s * pose_goal.position,
          pose_init.orientation.slerp(s, pose_goal.orientation),
        };
        state.setPositions(path[static_cast<std::size_t>(t - 1)]);
        pose_t const link_target = interpolate_tool_frame
          ? compose(target, inverse(context.robot.link_from_tool))
          : target;
        inverse_kinematics_initialization_e initialization =
          inverse_kinematics_initialization_e::swing;
        if (context.robot.ik_initializer) {
          std::optional<Eigen::VectorXd> const seed =
            context.robot.ik_initializer(
              state, interpolation_link, link_target);
          if (
            seed.has_value() && seed->size() == q_init.size() &&
            seed->allFinite()) {
            state.setPositions(*seed);
            initialization = inverse_kinematics_initialization_e::provided;
          }
        }
        inverse_kinematics_result_t const ik = solve_inverse_kinematics(
          inverse_kinematics_problem_t {
            .initial_state = state,
            .link = interpolation_link,
            .frame_from_link = link_target,
            .position_only = false,
          },
          inverse_kinematics_config_t {
            .max_iters = context.config.ik_max_iters,
            .tol = context.config.ik_tol,
            .initialization = initialization,
          });
        Eigen::VectorXd const q_linear = q_init + s * (q_goal - q_init);
        bool use_ik = ik.status == solve_status_e::success;
        if (use_ik && context.mode == planning_mode_e::grasped) {
          Eigen::VectorXd const jump =
            (ik.positions - path[static_cast<std::size_t>(t - 1)]).cwiseAbs();
          for (Eigen::Index j = 0; j < jump.size(); ++j) {
            if (jump(j) > std::max(5.0 * q_per_step(j), Scalar {0.25})) {
              use_ik = false;
              break;
            }
          }
        }
        path[static_cast<std::size_t>(t)] =
          clamp(use_ik ? ik.positions : q_linear, context.bounds);
      }
      return path;
    }

    std::vector<moving_body_t> moving_bodies(
      solver_context_t const& context, KinematicSnapshot const& kinematics) {
      std::vector<moving_body_t> result;
      result.reserve(
        context.robot.collision_bodies.size() +
        (context.attachment.has_value() ? 1U : 0U));
      for (motion_link_body_t const& binding : context.robot.collision_bodies) {
        pose_t const& pose = kinematics.frameFromLink(binding.link);
        result.push_back(moving_body_t {
          .body = BodyInstance {body_instance_config_t {
            .id = binding.entity,
            .model = binding.body_model,
            .frame_from_body = pose,
            .motion = {},
            .mobility = mobility_e::kinematic,
          }},
          .jac = body_pose_jacobian(kinematics, binding.link, pose_t {}),
        });
      }
      if (context.attachment.has_value()) {
        attachment_t const& attachment = *context.attachment;
        BodyInstance const& source = context.scene.body(attachment.body);
        result.push_back(moving_body_t {
          .body = BodyInstance {body_instance_config_t {
            .id = attachment.body,
            .model = source.modelPtr(),
            .frame_from_body = compose(
              kinematics.frameFromLink(attachment.link),
              attachment.link_from_body),
            .motion = {},
            .mobility = mobility_e::kinematic,
          }},
          .jac = body_pose_jacobian(
            kinematics, attachment.link, attachment.link_from_body),
        });
      }
      return result;
    }

    void add_collision_terms(
      solver_context_t const& context, KinematicSnapshot const& kinematics,
      int t, int steps, Scalar dt, Scalar& cost,
      Eigen::Ref<Eigen::VectorXd> grad, path_evaluation_t& evaluation,
      collision_alm_state_t const* alm) {
      std::vector<moving_body_t> moving = moving_bodies(context, kinematics);
      if (moving.empty()) {
        return;
      }

      std::unordered_map<EntityId, Matrix6X const*> jac_by_entity;
      std::vector<BodyInstance> bodies;
      std::vector<EntityId> ids;
      bodies.reserve(moving.size() + context.scene.bodyCount());
      ids.reserve(moving.size() + context.scene.bodyCount());
      for (moving_body_t const& value : moving) {
        jac_by_entity.emplace(value.body.id(), &value.jac);
        ids.push_back(value.body.id());
        bodies.emplace_back(body_instance_config_t {
          .id = value.body.id(),
          .model = value.body.modelPtr(),
          .frame_from_body = value.body.frameFromBody(),
          .motion = {},
          .mobility = mobility_e::kinematic,
        });
      }

      std::vector<EntityId> obstacle_ids;
      for (EntityId id : context.scene.entityIds()) {
        if (context.attachment.has_value() && id == context.attachment->body) {
          continue;
        }
        BodyInstance const& body = context.scene.body(id);
        obstacle_ids.push_back(id);
        ids.push_back(id);
        bodies.emplace_back(body_instance_config_t {
          .id = id,
          .model = body.modelPtr(),
          .frame_from_body = body.frameFromBody(),
          .motion = body.motion(),
          .mobility = body.mobility(),
        });
      }

      auto snapshot = std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
        .frame = context.scene.frame(),
        .bodies = std::move(bodies),
      });
      SceneView const view {snapshot, ids};
      std::vector<collision_body_pair_t> body_pairs;
      for (moving_body_t const& value : moving) {
        bool const is_target = context.attachment.has_value() &&
          value.body.id() == context.attachment->body;
        if (
          is_target &&
          !(t >= grasped_collision_boundary &&
            (t <= steps - grasped_collision_boundary || t == steps - 1))) {
          continue;
        }
        for (EntityId obstacle : obstacle_ids) {
          body_pairs.push_back(
            collision_body_pair_t {value.body.id(), obstacle});
        }
      }
      body_pairs.insert(
        body_pairs.end(), context.robot.self_collision_pairs.begin(),
        context.robot.self_collision_pairs.end());

      std::vector<collision_pair_t> const geometry_pairs =
        brute_force_middle_phase(view, body_pairs);
      // One PHR constraint on the representative (deepest) feature of each
      // body pair, plus a clearance term on every geometry feature, as in
      // legacy process_pair. Stable entity IDs replace its gripper-side IDs.
      std::map<collision_key_t, collision_constraint_t> constraints;
      for (collision_pair_t const& pair : geometry_pairs) {
        Geometry const& first_geometry = snapshot->body(pair.first.entity)
                                           .model()
                                           .geometry(pair.first.geometry);
        Geometry const& second_geometry = snapshot->body(pair.second.entity)
                                            .model()
                                            .geometry(pair.second.geometry);
        bool const involves_plane =
          first_geometry.type() == geometry_type_e::plane ||
          second_geometry.type() == geometry_type_e::plane;
        Scalar feasibility_margin =
          involves_plane ? context.config.plane_feasibility_margin : 0.0;
        bool const involves_target = context.attachment.has_value() &&
          (pair.first.entity == context.attachment->body ||
           pair.second.entity == context.attachment->body);
        if (involves_target && !involves_plane) {
          feasibility_margin = -context.config.target_collision_tol;
        }
        auto record_violation = [&](Scalar gap) {
          Scalar const violation = feasibility_margin - gap;
          evaluation.max_collision_violation =
            std::max(evaluation.max_collision_violation, violation);
        };
        auto record_constraint = [&](Scalar gap, Eigen::RowVectorXd d_gap) {
          if (alm == nullptr) {
            return;
          }
          collision_key_t const key {
            t, std::min(pair.first.entity, pair.second.entity),
            std::max(pair.first.entity, pair.second.entity)};
          Scalar const c = feasibility_margin - gap;
          auto const it = constraints.find(key);
          if (it == constraints.end() || c > it->second.violation) {
            constraints.insert_or_assign(
              key, collision_constraint_t {c, std::move(d_gap)});
          }
        };

        diffable_contact_feature_t feature;
        try {
          feature = compute_diffable_contact(*snapshot, pair);
        } catch (std::invalid_argument const&) {
          contact_feature_t const contact = compute_contact(*snapshot, pair);
          record_violation(contact.gap);
          record_constraint(contact.gap, Eigen::RowVectorXd::Zero(grad.size()));
          continue;
        }
        Scalar const penetration =
          std::max(context.config.collision_margin - feature.gap, Scalar {0.0});
        Scalar const clamp = context.config.collision_penetration_clamp;
        Scalar const pen_grad = clamp > 0.0 ? std::min(penetration, clamp) : penetration;
        Scalar const pen_cost = pen_grad * (penetration - 0.5 * pen_grad);
        Scalar const weight = dt * context.config.collision_weight *
          (alm != nullptr ? 0.1 : 1.0);
        cost += weight * pen_cost;
        Eigen::RowVectorXd d_gap = Eigen::RowVectorXd::Zero(grad.size());
        auto const first = jac_by_entity.find(pair.first.entity);
        if (first != jac_by_entity.end()) {
          d_gap += feature.d_gap.leftCols<6>() * *first->second;
        }
        auto const second = jac_by_entity.find(pair.second.entity);
        if (second != jac_by_entity.end()) {
          d_gap += feature.d_gap.rightCols<6>() * *second->second;
        }
        grad -= weight * pen_grad * d_gap.transpose();

        record_violation(feature.gap);
        record_constraint(feature.gap, std::move(d_gap));
      }
      for (auto const& [key, constraint] : constraints) {
        auto const term = planner_detail::collision_alm_term(
          alm->multiplier(key), alm->beta, constraint.violation);
        cost += term.cost;
        grad -= term.d_violation * constraint.d_gap.transpose();
        evaluation.collision_violations.emplace(key, constraint.violation);
      }
    }

    path_evaluation_t evaluate_path(
      solver_context_t const& context, std::vector<Eigen::VectorXd> const& path,
      Eigen::VectorXd const& q_init, Eigen::VectorXd const& q_goal,
      pose_t const& tool_goal, collision_alm_state_t const* alm = nullptr) {
      int const steps = static_cast<int>(path.size());
      int const dof = static_cast<int>(q_init.size());
      Scalar const dt = 1.0 / steps;
      Scalar const T_half = std::max(1.0, (steps - 1) * 0.5);
      auto smooth_weight = [&](int i) {
        Scalar const dist = std::min(i, steps - 2 - i);
        return dt * context.config.smooth_weight *
          std::exp(
                 context.config.smooth_boundary_alpha * (1.0 - dist / T_half));
      };

      path_evaluation_t result;
      result.grad = Eigen::VectorXd::Zero(steps * dof);
      auto grad_at = [&](int t) { return result.grad.segment(t * dof, dof); };

      Eigen::VectorXd const dq_start =
        wrapped_difference(q_init, path.front(), context.bounds);
      result.cost +=
        0.5 * context.config.boundary_weight * dq_start.squaredNorm();
      grad_at(0) -= context.config.boundary_weight * dq_start;
      for (int t = 0; t < steps - 1; ++t) {
        Eigen::VectorXd const dq = wrapped_difference(
          path[static_cast<std::size_t>(t + 1)],
          path[static_cast<std::size_t>(t)], context.bounds);
        Eigen::VectorXd weighted = dq;
        if (weighted.size() > 0) {
          weighted(0) *= context.config.swing_smoothness_scale;
        }
        Scalar const weight = smooth_weight(t);
        result.cost += 0.5 * weight * dq.dot(weighted);
        grad_at(t) -= weight * weighted;
        grad_at(t + 1) += weight * weighted;
      }
      if (context.mode == planning_mode_e::free) {
        Eigen::VectorXd const dq_goal =
          wrapped_difference(q_goal, path.back(), context.bounds);
        result.cost +=
          0.5 * context.config.boundary_weight * dq_goal.squaredNorm();
        grad_at(steps - 1) -= context.config.boundary_weight * dq_goal;
      }

      KinematicState state = context.robot.initial_state;
      for (int t = 0; t < steps; ++t) {
        Eigen::VectorXd const& q = path[static_cast<std::size_t>(t)];
        for (Eigen::Index j = 0; j < q.size(); ++j) {
          if (
            !std::isfinite(context.bounds.lower(j)) ||
            !std::isfinite(context.bounds.upper(j))) {
            continue;
          }
          Scalar const upper = std::max(
            q(j) - context.bounds.upper(j) + context.config.joint_limit_margin,
            Scalar {0.0});
          Scalar const lower = std::max(
            context.bounds.lower(j) + context.config.joint_limit_margin - q(j),
            Scalar {0.0});
          Scalar const violation = upper + lower;
          result.cost += 0.5 * dt * context.config.joint_limit_weight *
            violation * violation;
          grad_at(t)(j) +=
            dt * context.config.joint_limit_weight * (upper - lower);
        }

        state.setPositions(q);
        KinematicSnapshot const snapshot = forward_kinematics(state);
        if (context.mode == planning_mode_e::grasped && t == steps - 1) {
          pose_t const achieved = tool_pose(snapshot, context.robot);
          Vector6 error = Vector6::Zero();
          error.head<3>() = context.config.grasped_boundary_pos_scale *
            (achieved.position - tool_goal.position);
          error.tail<3>() = context.config.grasped_boundary_rot_scale *
            orientation_error(tool_goal.orientation, achieved.orientation);
          Matrix6X J = hybrid_tool_jacobian(snapshot, context.robot);
          J.topRows<3>() *= context.config.grasped_boundary_pos_scale;
          J.bottomRows<3>() *= context.config.grasped_boundary_rot_scale;
          result.cost +=
            0.5 * context.config.boundary_weight * error.squaredNorm();
          grad_at(t) += context.config.boundary_weight * J.transpose() * error;
        }
        add_collision_terms(
          context, snapshot, t, steps, dt, result.cost, grad_at(t),
          result, alm);
      }
      return result;
    }

    segment_result_t solve_segment(
      solver_context_t const& context, Eigen::VectorXd const& q_init,
      Eigen::VectorXd const& q_goal, LinkId interpolation_link, int steps,
      bool workspace_init, bool interpolate_tool_frame) {
      std::vector<Eigen::VectorXd> path = initialize_path(
        context, q_init, q_goal, interpolation_link, steps, workspace_init,
        interpolate_tool_frame);
      KinematicState goal_state = context.robot.initial_state;
      goal_state.setPositions(q_goal);
      pose_t const tool_goal =
        tool_pose(forward_kinematics(goal_state), context.robot);

      Scalar const dt = 1.0 / steps;
      Scalar const T_half = std::max(1.0, (steps - 1) * 0.5);
      auto smooth_weight = [&](int i) {
        Scalar const dist = std::min(i, steps - 2 - i);
        return dt * context.config.smooth_weight *
          std::exp(
                 context.config.smooth_boundary_alpha * (1.0 - dist / T_half));
      };
      Scalar const goal_scale = std::max(
        context.config.grasped_boundary_pos_scale *
          context.config.grasped_boundary_pos_scale,
        context.config.grasped_boundary_rot_scale *
          context.config.grasped_boundary_rot_scale);
      Eigen::VectorXd boundary_diag = Eigen::VectorXd::Zero(steps);
      boundary_diag(0) = context.config.boundary_weight;
      boundary_diag(steps - 1) = context.config.boundary_weight *
        (context.mode == planning_mode_e::free ? 1.0 : goal_scale);
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(steps, steps);
      for (int t = 0; t < steps; ++t) {
        Scalar diagonal = boundary_diag(t);
        if (t > 0) {
          diagonal += smooth_weight(t - 1);
        }
        if (t < steps - 1) {
          diagonal += smooth_weight(t);
        }
        A(t, t) = diagonal;
      }
      for (int t = 0; t < steps - 1; ++t) {
        A(t, t + 1) = -smooth_weight(t);
        A(t + 1, t) = -smooth_weight(t);
      }
      Eigen::LDLT<Eigen::MatrixXd> const A_solver {A};
      Eigen::MatrixXd A_swing = A;
      for (int t = 0; t < steps - 1; ++t) {
        Scalar const delta =
          (context.config.swing_smoothness_scale - 1.0) * smooth_weight(t);
        A_swing(t, t) += delta;
        A_swing(t + 1, t + 1) += delta;
        A_swing(t, t + 1) -= delta;
        A_swing(t + 1, t) -= delta;
      }
      Eigen::LDLT<Eigen::MatrixXd> const A_swing_solver {A_swing};

      int const dof = static_cast<int>(q_init.size());
      Scalar alpha = context.config.step_size;
      Scalar path_change = std::numeric_limits<Scalar>::infinity();
      int iters = 0;
      bool converged = false;
      collision_alm_state_t alm_state {
        .beta = context.config.collision_alm_beta_init > 0.0
          ? context.config.collision_alm_beta_init : context.config.collision_weight,
        .duals = {},
      };
      collision_alm_state_t const* alm =
        context.config.collision_alm_enabled ? &alm_state : nullptr;
      int const outer_max = alm != nullptr ? context.config.collision_alm_max_iters : 1;
      Scalar prev_violation = std::numeric_limits<Scalar>::infinity();
      path_evaluation_t path_eval;
      for (int outer = 0; outer < outer_max; ++outer) {
        // Restart FISTA for the changed augmented objective, retaining the
        // accepted path and line-search step as the legacy outer loop does.
        std::vector<Eigen::VectorXd> y = path;
        std::vector<Eigen::VectorXd> path_new(static_cast<std::size_t>(steps));
        Scalar momentum_prev = 1.0;
        converged = false;
        path_eval = evaluate_path(context, path, q_init, q_goal, tool_goal, alm);
        for (int iter = 0; iter < context.config.max_iters; ++iter) {
          path_evaluation_t const eval_y =
            evaluate_path(context, y, q_init, q_goal, tool_goal, alm);
          Eigen::MatrixXd grad(steps, dof);
          for (int t = 0; t < steps; ++t) {
            grad.row(t) = eval_y.grad.segment(t * dof, dof).transpose();
          }
          Eigen::MatrixXd natural_grad = A_solver.solve(grad);
          if (dof > 0 && context.config.swing_smoothness_scale != 1.0) {
            natural_grad.col(0) = A_swing_solver.solve(grad.col(0));
          }

          alpha *= bt_grow;
          path_evaluation_t eval_new = eval_y;
          for (int bt = 0; bt < bt_max; ++bt) {
            Eigen::MatrixXd dx = Eigen::MatrixXd::Zero(steps, dof);
            Scalar grad_dot_dx = 0.0;
            for (int t = 0; t < steps; ++t) {
              bool const pinned = t == 0 ||
                (context.mode == planning_mode_e::free && t == steps - 1);
              if (pinned) {
                path_new[static_cast<std::size_t>(t)] =
                  y[static_cast<std::size_t>(t)];
              } else {
                path_new[static_cast<std::size_t>(t)] = clamp(
                  y[static_cast<std::size_t>(t)] -
                    alpha * natural_grad.row(t).transpose(),
                  context.bounds);
              }
              dx.row(t) = (path_new[static_cast<std::size_t>(t)] -
                           y[static_cast<std::size_t>(t)])
                            .transpose();
              grad_dot_dx +=
                eval_y.grad.segment(t * dof, dof).dot(dx.row(t).transpose());
            }
            Scalar metric_norm_sq = 0.0;
            for (int t = 0; t < steps; ++t) {
              Scalar diagonal = boundary_diag(t);
              if (t > 0) {
                diagonal += smooth_weight(t - 1);
              }
              if (t < steps - 1) {
                diagonal += smooth_weight(t);
              }
              metric_norm_sq += diagonal * dx.row(t).squaredNorm();
              if (t > 0) {
                metric_norm_sq -=
                  smooth_weight(t - 1) * dx.row(t).dot(dx.row(t - 1));
              }
              if (t < steps - 1) {
                metric_norm_sq -= smooth_weight(t) * dx.row(t).dot(dx.row(t + 1));
              }
            }
            if (dof > 0 && context.config.swing_smoothness_scale != 1.0) {
              for (int t = 0; t < steps - 1; ++t) {
                Scalar const delta = dx(t + 1, 0) - dx(t, 0);
                metric_norm_sq += (context.config.swing_smoothness_scale - 1.0) *
                  smooth_weight(t) * delta * delta;
              }
            }
            eval_new =
              evaluate_path(context, path_new, q_init, q_goal, tool_goal, alm);
            Scalar const rhs =
              eval_y.cost + grad_dot_dx + metric_norm_sq / (2.0 * alpha);
            if (eval_new.cost <= rhs + 1e-9) {
              break;
            }
            alpha *= bt_shrink;
          }

          Scalar const momentum_new =
            0.5 * (1.0 + std::sqrt(1.0 + 4.0 * momentum_prev * momentum_prev));
          Scalar const beta = (momentum_prev - 1.0) / momentum_new;
          bool const restart = eval_new.cost > path_eval.cost;
          path_change = 0.0;
          for (int t = 0; t < steps; ++t) {
            Eigen::VectorXd delta = wrapped_difference(
              path_new[static_cast<std::size_t>(t)],
              path[static_cast<std::size_t>(t)], context.bounds);
            path_change = std::max(path_change, delta.cwiseAbs().maxCoeff());
            bool const pinned =
              t == 0 || (context.mode == planning_mode_e::free && t == steps - 1);
            if (pinned || restart) {
              y[static_cast<std::size_t>(t)] =
                path_new[static_cast<std::size_t>(t)];
            } else {
              y[static_cast<std::size_t>(t)] = clamp(
                path_new[static_cast<std::size_t>(t)] + beta * delta,
                context.bounds);
            }
          }
          momentum_prev = restart ? 1.0 : momentum_new;
          path = path_new;
          path_eval = std::move(eval_new);
          ++iters;
          if (path_change < context.config.tol) {
            converged = true;
            break;
          }
        }

        if (alm == nullptr) {
          break;
        }
        // path_eval is the measurement at the accepted path. Trial-point
        // evaluations never mutate multipliers or the penalty parameter.
        Scalar const violation = path_eval.max_collision_violation;
        alm_state.update(path_eval.collision_violations);
        if (violation <= context.config.collision_alm_tol || outer + 1 == outer_max) {
          break;
        }
        if (violation > 0.5 * prev_violation) {
          Scalar const next_beta =
            alm_state.beta * context.config.collision_alm_beta_increase;
          if (!std::isfinite(next_beta)) {
            break;
          }
          alm_state.beta = next_beta;
        }
        prev_violation = violation;
      }
      for (Eigen::Index j = 0; j < q_init.size(); ++j) {
        if (!context.bounds.full_revolution[static_cast<std::size_t>(j)]) {
          continue;
        }
        for (int t = 1; t < steps; ++t) {
          Scalar const diff = path[static_cast<std::size_t>(t)](j) -
            path[static_cast<std::size_t>(t - 1)](j);
          path[static_cast<std::size_t>(t)](j) =
            path[static_cast<std::size_t>(t - 1)](j) + diff -
            2.0 * std::numbers::pi *
              std::round(diff / (2.0 * std::numbers::pi));
          path[static_cast<std::size_t>(t)] =
            clamp(path[static_cast<std::size_t>(t)], context.bounds);
        }
      }

      path_eval = evaluate_path(context, path, q_init, q_goal, tool_goal);
      bool feasible = path_eval.max_collision_violation <= 0.0;
      KinematicModel const& model = context.robot.initial_state.model();
      for (Eigen::VectorXd const& q : path) {
        feasible = feasible && model.positionsWithinLimits(q);
      }
      std::vector<std::optional<pose_t>> target_path(
        static_cast<std::size_t>(steps));
      if (context.attachment.has_value()) {
        KinematicState state = context.robot.initial_state;
        for (int t = 0; t < steps; ++t) {
          state.setPositions(path[static_cast<std::size_t>(t)]);
          KinematicSnapshot const snapshot = forward_kinematics(state);
          target_path[static_cast<std::size_t>(t)] = compose(
            snapshot.frameFromLink(context.attachment->link),
            context.attachment->link_from_body);
        }
      }

      return segment_result_t {
        .path = std::move(path),
        .target_path = std::move(target_path),
        .solver = {
          .iters = iters,
          .converged = converged,
          .objective = path_eval.cost,
          .grad_norm = path_change,
        },
        .feasible = feasible,
        .failure = feasible
          ? planner_failure_t {}
          : planner_failure_t {
              .code = "collision_or_joint_limit",
              .message =
                "optimized motion violates a collision or joint constraint",
              .retryable = true,
            },
      };
    }

    motion_result_t solve_motion(
      SceneView const& scene, motion_robot_t const& robot,
      std::span<motion_waypoint_t const> waypoints,
      motion_planning_config_t const& config, planning_mode_e mode,
      std::optional<attachment_t> attachment) {
      if (!valid_config(config)) {
        return invalid_result(
          "invalid_config", "motion-planning configuration is invalid");
      }
      KinematicModel const& model = robot.initial_state.model();
      if (
        robot.initial_state.frame() != scene.frame() ||
        model.findLink(robot.tool_link) == nullptr ||
        !is_valid(robot.link_from_tool)) {
        return invalid_result(
          "invalid_robot",
          "motion robot must use the scene frame and a valid tool link");
      }
      if (waypoints.empty()) {
        return invalid_result(
          "missing_waypoints",
          "motion planning requires at least one waypoint");
      }
      if (!model.positionsWithinLimits(robot.initial_state.positions())) {
        return invalid_result(
          "initial_state_out_of_joint_limits",
          "motion initial state violates a joint limit");
      }

      std::unordered_set<EntityId> collision_entities;
      for (motion_link_body_t const& binding : robot.collision_bodies) {
        if (
          model.findLink(binding.link) == nullptr || !binding.entity.valid() ||
          binding.body_model == nullptr ||
          scene.snapshot().findBody(binding.entity) != nullptr ||
          !collision_entities.emplace(binding.entity).second) {
          return invalid_result(
            "invalid_collision_binding",
            "robot collision bindings must be valid, unique, and outside the "
            "scene");
        }
      }
      if (attachment.has_value()) {
        if (
          model.findLink(attachment->link) == nullptr ||
          scene.findBody(attachment->body) == nullptr ||
          !is_valid(attachment->link_from_body)) {
          return invalid_result(
            "invalid_attachment",
            "grasped motion attachment must reference the robot and scene");
        }
      }
      for (collision_body_pair_t const& pair : robot.self_collision_pairs) {
        if (
          pair.first == pair.second ||
          !collision_entities.contains(pair.first) ||
          !collision_entities.contains(pair.second)) {
          return invalid_result(
            "invalid_self_collision_pair",
            "self-collision pairs must reference two distinct robot collision "
            "bindings");
        }
      }
      for (motion_waypoint_t const& waypoint : waypoints) {
        if (waypoint.steps < 2) {
          return invalid_result(
            "invalid_waypoint_steps",
            "each motion waypoint requires at least two trajectory samples");
        }
      }

      joint_bounds_t bounds = make_joint_bounds(model);
      if (!bounds.valid) {
        return invalid_result(
          "invalid_joint_limits", "motion model has inconsistent joint limits");
      }
      solver_context_t const context {
        .scene = scene,
        .robot = robot,
        .config = config,
        .mode = mode,
        .attachment = attachment,
        .bounds = std::move(bounds),
      };

      motion_result_t result {
        .status = solve_status_e::success,
        .trajectory = {},
        .solver = {.iters = 0, .converged = true},
        .failure = {},
      };
      Eigen::VectorXd q_initial = robot.initial_state.positions();
      for (motion_waypoint_t const& waypoint : waypoints) {
        LinkId interpolation_link;
        bool interpolate_tool_frame = false;
        planner_failure_t failure;
        std::optional<Eigen::VectorXd> const q_goal = resolve_goal(
          context, q_initial, waypoint.goal, interpolation_link,
          interpolate_tool_frame, failure);
        if (!q_goal.has_value()) {
          result.status = solve_status_e::infeasible;
          result.failure = std::move(failure);
          return result;
        }
        bool const workspace_init = mode == planning_mode_e::grasped ||
          std::holds_alternative<link_pose_goal_t>(waypoint.goal);
        segment_result_t segment = solve_segment(
          context, q_initial, *q_goal, interpolation_link, waypoint.steps,
          workspace_init, interpolate_tool_frame);
        result.solver.iters += segment.solver.iters;
        result.solver.converged =
          result.solver.converged && segment.solver.converged;
        result.solver.objective += segment.solver.objective;
        result.solver.grad_norm =
          std::max(result.solver.grad_norm, segment.solver.grad_norm);
        for (std::size_t i = 0; i < segment.path.size(); ++i) {
          result.trajectory.samples.push_back(trajectory_sample_t {
            .robot =
              robot_state_t {
                .positions = segment.path[i],
                .gripper_opening = robot.gripper_opening,
              },
            .frame_from_target = segment.target_path[i],
          });
        }
        if (!segment.feasible) {
          result.status = solve_status_e::infeasible;
          result.failure = std::move(segment.failure);
          return result;
        }
        q_initial = segment.path.back();
      }
      return result;
    }

  }  // namespace

  motion_result_t solve_free_motion(
    free_motion_problem_t const& problem,
    motion_planning_config_t const& config) {
    return solve_motion(
      problem.scene, problem.robot, problem.waypoints, config,
      planning_mode_e::free, std::nullopt);
  }

  motion_result_t solve_grasped_motion(
    grasped_motion_problem_t const& problem,
    motion_planning_config_t const& config) {
    return solve_motion(
      problem.scene, problem.robot, problem.waypoints, config,
      planning_mode_e::grasped, problem.attachment);
  }

}  // namespace stacking_core
