#include "evaluation.hpp"

#include <stacking_core/planner/grasp/simulation.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace stacking_core {
  namespace {

    using grasp_detail::matrix3x7_t;
    using grasp_detail::matrix_x7_t;
    using grasp_detail::vector7_t;

    struct pgs_contact_t {
      grasp_detail::grasp_dynamics_contact_t contact;
      Vector3 tangent_0 = Vector3::Zero();
      Vector3 tangent_1 = Vector3::Zero();
      Eigen::Vector3d impulse = Eigen::Vector3d::Zero();
    };

    grasp_simulation_result_t invalid_result(
      std::string code, std::string message) {
      return grasp_simulation_result_t {
        .status = solve_status_e::invalid_problem,
        .grasp = {},
        .trajectory = {},
        .contacts = {},
        .terminal_velocity_norm = 0.0,
        .settled = false,
        .left_contact = false,
        .right_contact = false,
        .failure =
          planner_failure_t {
            .code = std::move(code),
            .message = std::move(message),
            .retryable = false,
          },
      };
    }

    bool valid_config(grasp_simulation_config_t const& config) {
      return config.steps > 0 && std::isfinite(config.dt) && config.dt > 0.0 &&
        std::isfinite(config.virtual_closing_effort) &&
        config.virtual_closing_effort >= 0.0 &&
        std::isfinite(config.virtual_approach_force) &&
        config.virtual_approach_force >= 0.0 && std::isfinite(config.damping) &&
        config.damping >= 0.0 && std::isfinite(config.friction) &&
        config.friction >= 0.0 && config.pgs_iters > 0 &&
        std::isfinite(config.error_reduction_ratio) &&
        config.error_reduction_ratio >= 0.0 &&
        std::isfinite(config.target_contact_margin) &&
        config.target_contact_margin >= 0.0 &&
        std::isfinite(config.obstacle_margin) &&
        config.obstacle_margin >= 0.0 &&
        std::isfinite(config.plane_obstacle_margin) &&
        config.plane_obstacle_margin >= 0.0 &&
        std::isfinite(config.active_impulse_tol) &&
        config.active_impulse_tol >= 0.0 &&
        std::isfinite(config.settled_velocity_tol) &&
        config.settled_velocity_tol >= 0.0 &&
        std::isfinite(config.relaxed_velocity_tol) &&
        config.relaxed_velocity_tol >= config.settled_velocity_tol &&
        config.grasp_offset_tol.allFinite() &&
        (config.grasp_offset_tol.array() >= 0.0).all();
    }

    std::pair<Vector3, Vector3> tangents(Vector3 const& normal) {
      Vector3 const axis =
        std::abs(normal.z()) < 0.9 ? Vector3::UnitZ() : Vector3::UnitY();
      Vector3 const tangent_0 = normal.cross(axis).normalized();
      return {tangent_0, normal.cross(tangent_0).normalized()};
    }

    Eigen::Vector3d project_cone(Eigen::Vector3d impulse, Scalar friction) {
      impulse.x() = std::max(0.0, impulse.x());
      Eigen::Vector2d tangent = impulse.tail<2>();
      Scalar const norm = tangent.norm();
      Scalar const limit = friction * impulse.x();
      if (norm > limit) {
        if (limit <= 0.0) {
          tangent.setZero();
        } else {
          tangent *= limit / norm;
        }
      }
      impulse.tail<2>() = tangent;
      return impulse;
    }

    Eigen::Matrix<Scalar, 3, 7> contact_jacobian(pgs_contact_t const& contact) {
      Eigen::Matrix<Scalar, 3, 7> J;
      J.row(0) = contact.contact.d_gap.transpose();
      J.row(1) = contact.tangent_0.transpose() * contact.contact.d_point_first;
      J.row(2) = contact.tangent_1.transpose() * contact.contact.d_point_first;
      return J;
    }

    Eigen::VectorXd solve_pgs(
      Eigen::MatrixXd const& G, Eigen::VectorXd const& g, Scalar friction,
      int max_iters) {
      constexpr Scalar eps = 1e-9;
      Eigen::VectorXd impulse = Eigen::VectorXd::Zero(g.size());
      Eigen::Index const count = g.size() / 3;
      std::vector<std::array<Scalar, 3>> diag_inv(
        static_cast<std::size_t>(count));
      for (Eigen::Index i = 0; i < count; ++i) {
        for (Eigen::Index axis = 0; axis < 3; ++axis) {
          Scalar const value = G(3 * i + axis, 3 * i + axis);
          diag_inv[static_cast<std::size_t>(i)]
                  [static_cast<std::size_t>(axis)] =
                    value > eps ? 1.0 / value : 0.0;
        }
      }
      for (int iter = 0; iter < max_iters; ++iter) {
        for (Eigen::Index i = 0; i < count; ++i) {
          Eigen::Vector3d local = impulse.segment<3>(3 * i);
          Eigen::Vector3d const residual =
            G.middleRows<3>(3 * i) * impulse + g.segment<3>(3 * i);
          for (Eigen::Index axis = 0; axis < 3; ++axis) {
            local(axis) -= residual(axis) *
              diag_inv[static_cast<std::size_t>(i)]
                      [static_cast<std::size_t>(axis)];
          }
          impulse.segment<3>(3 * i) = project_cone(local, friction);
        }
      }
      return impulse;
    }

    planner_failure_t infeasible_failure(
      bool settled, bool left, bool right, Vector3 const& offset,
      Vector3 const& tol) {
      std::string primary = "simulation_unsettled";
      if (settled && !(left && right)) {
        primary = "simulation_missing_both_side_contact";
      } else if (settled && std::abs(offset.x()) > tol.x()) {
        primary = "simulation_x_offset";
      } else if (settled && std::abs(offset.y()) > tol.y()) {
        primary = "simulation_y_offset";
      } else if (settled && std::abs(offset.z()) > tol.z()) {
        primary = "simulation_z_offset";
      }
      return planner_failure_t {
        .code = std::move(primary),
        .message =
          "grasp simulation did not settle into a centered "
          "two-sided contact",
        .retryable = true,
      };
    }

  }  // namespace

  grasp_simulation_result_t simulate_grasp(
    grasp_simulation_problem_t const& problem,
    grasp_simulation_config_t const& config) {
    if (!valid_config(config)) {
      return invalid_result(
        "invalid_grasp_simulation_config",
        "grasp simulation configuration is invalid");
    }
    grasp_generation_config_t validation_config;
    validation_config.contact_margin = config.target_contact_margin;
    validation_config.separate_margin = config.obstacle_margin;
    validation_config.plane_separate_margin = config.plane_obstacle_margin;
    grasp_problem_t const grasp_problem {
      .phases = {problem.phase},
      .gripper = problem.gripper,
      .seed = problem.initial_grasp,
    };
    if (
      auto const failure = grasp_detail::validate_grasp_problem(
        grasp_problem, validation_config)) {
      return invalid_result(failure->code, failure->message);
    }

    grasp_simulation_result_t out;
    out.status = solve_status_e::infeasible;
    out.grasp = problem.initial_grasp;
    out.trajectory.reserve(static_cast<std::size_t>(config.steps));
    vector7_t velocity = vector7_t::Zero();
    Scalar const inertia_inv = 1.0 / (1.0 + config.dt * config.damping);
    std::vector<pgs_contact_t> active_target;

    for (int step = 0; step < config.steps; ++step) {
      out.trajectory.push_back(out.grasp);
      std::vector<grasp_detail::grasp_dynamics_contact_t> const detected =
        grasp_detail::evaluate_grasp_dynamics_contacts(
          grasp_problem, out.grasp,
          grasp_detail::grasp_dynamics_config_t {
            .target_margin = config.target_contact_margin,
            .obstacle_margin = config.obstacle_margin,
            .plane_margin = config.plane_obstacle_margin,
          });
      std::vector<pgs_contact_t> contacts;
      contacts.reserve(detected.size());
      for (auto const& contact : detected) {
        Vector3 normal = contact.feature.normal;
        if (normal.squaredNorm() < 1e-12) {
          normal = Vector3::UnitZ();
        }
        normal.normalize();
        auto const [tangent_0, tangent_1] = tangents(normal);
        auto value = contact;
        value.feature.normal = normal;
        contacts.push_back(pgs_contact_t {
          .contact = std::move(value),
          .tangent_0 = tangent_0,
          .tangent_1 = tangent_1,
          .impulse = Eigen::Vector3d::Zero(),
        });
      }

      vector7_t effort = vector7_t::Zero();
      effort(6) = -config.virtual_closing_effort;
      effort.head<3>() = config.virtual_approach_force *
        (problem.phase.scene.body(problem.phase.target)
           .frameFromBody()
           .position -
         out.grasp.frame_from_grasp.position);
      vector7_t const velocity_free = velocity +
        config.dt * inertia_inv * (effort - config.damping * velocity);

      active_target.clear();
      out.contacts.clear();
      if (contacts.empty()) {
        velocity = velocity_free;
      } else {
        matrix_x7_t J(3 * static_cast<Eigen::Index>(contacts.size()), 7);
        Eigen::VectorXd g =
          Eigen::VectorXd::Zero(3 * static_cast<Eigen::Index>(contacts.size()));
        for (std::size_t i = 0; i < contacts.size(); ++i) {
          J.middleRows<3>(3 * static_cast<Eigen::Index>(i)) =
            config.dt * contact_jacobian(contacts[i]);
          g(3 * static_cast<Eigen::Index>(i)) =
            config.error_reduction_ratio * contacts[i].contact.feature.gap;
        }
        g += J * velocity_free;
        Eigen::MatrixXd const G = inertia_inv * J * J.transpose();
        Eigen::VectorXd const impulse =
          solve_pgs(G, g, config.friction, config.pgs_iters);
        velocity = velocity_free + inertia_inv * J.transpose() * impulse;
        for (std::size_t i = 0; i < contacts.size(); ++i) {
          pgs_contact_t& contact = contacts[i];
          contact.impulse =
            impulse.segment<3>(3 * static_cast<Eigen::Index>(i));
          Scalar const next_gap =
            contact.contact.feature.gap + contact.contact.d_gap.dot(velocity);
          if (
            !contact.contact.target_contact ||
            contact.impulse.x() <= config.active_impulse_tol ||
            std::abs(next_gap) > config.target_contact_margin) {
            continue;
          }
          active_target.push_back(contact);
          Vector3 const force_on_gripper =
            contact.impulse.x() * contact.contact.feature.normal +
            contact.impulse.y() * contact.tangent_0 +
            contact.impulse.z() * contact.tangent_1;
          out.contacts.push_back(grasp_contact_t {
            .feature = contact.contact.feature,
            .force = -force_on_gripper,
          });
        }
      }

      out.grasp = grasp_detail::apply_grasp_step(
        out.grasp, (config.dt * velocity).eval());
      out.grasp.opening = std::clamp(
        out.grasp.opening, problem.gripper.opening_lower,
        problem.gripper.opening_upper);
    }

    // Legacy refinement consumes the final stored pre-step state. Preserve
    // that convention so numerical comparisons do not gain one hidden step.
    out.grasp = out.trajectory.back();
    out.terminal_velocity_norm = velocity.norm();
    out.settled = out.terminal_velocity_norm <= config.settled_velocity_tol;
    for (pgs_contact_t const& contact : active_target) {
      if (!contact.contact.side.has_value()) {
        continue;
      }
      out.left_contact |= *contact.contact.side == grasp_contact_side_e::left;
      out.right_contact |= *contact.contact.side == grasp_contact_side_e::right;
    }
    Vector3 const offset = out.grasp.frame_from_grasp.orientation.conjugate() *
      (problem.phase.scene.body(problem.phase.target).frameFromBody().position -
       out.grasp.frame_from_grasp.position);
    bool const centered =
      (offset.cwiseAbs().array() <= config.grasp_offset_tol.array()).all();
    bool const both_sides = out.left_contact && out.right_contact;
    bool const relaxed = config.accept_relaxed && !out.settled &&
      out.terminal_velocity_norm <= config.relaxed_velocity_tol && centered &&
      both_sides;
    if ((out.settled || relaxed) && centered && both_sides) {
      out.status = solve_status_e::success;
      return out;
    }
    out.failure = infeasible_failure(
      out.settled, out.left_contact, out.right_contact, offset,
      config.grasp_offset_tol);
    return out;
  }

}  // namespace stacking_core
