#pragma once

#include <stacking_core/planner/grasp.hpp>

#include <Eigen/Core>

#include <optional>
#include <string>

namespace stacking_core::grasp_detail {

  using vector7_t = Eigen::Matrix<Scalar, 7, 1>;
  using matrix67_t = Eigen::Matrix<Scalar, 6, 7>;
  using matrix7_t = Eigen::Matrix<Scalar, 7, 7>;
  using matrix3x7_t = Eigen::Matrix<Scalar, 3, 7>;
  using matrix_x7_t = Eigen::Matrix<Scalar, Eigen::Dynamic, 7>;

  struct grasp_evaluation_t {
    Scalar objective = 0.0;
    Scalar score = 0.0;
    vector7_t grad = vector7_t::Zero();
    Eigen::VectorXd c_ineq;
    Eigen::VectorXd c_eq;
    matrix_x7_t jac_ineq;
    matrix_x7_t jac_eq;
    std::vector<grasp_contact_t> contacts;

    [[nodiscard]] bool finite() const noexcept;
  };

  struct joint_evaluation_t {
    Scalar objective = 0.0;
    Scalar score = 0.0;
    Eigen::VectorXd grad;
    Eigen::VectorXd c_ineq;
    Eigen::VectorXd c_eq;
    Eigen::MatrixXd jac_ineq;
    Eigen::MatrixXd jac_eq;
    grasp_t grasp;
    std::vector<grasp_contact_t> contacts;

    [[nodiscard]] bool finite() const noexcept;
  };

  struct grasp_dynamics_config_t {
    Scalar target_margin = 0.0;
    Scalar obstacle_margin = 0.0;
    Scalar plane_margin = 0.0;
  };

  struct grasp_dynamics_contact_t {
    diffable_contact_feature_t feature;
    matrix3x7_t d_point_first = matrix3x7_t::Zero();
    vector7_t d_gap = vector7_t::Zero();
    bool target_contact = false;
    std::optional<grasp_contact_side_e> side;
  };

  [[nodiscard]] std::optional<planner_failure_t> validate_grasp_problem(
    grasp_problem_t const& problem, grasp_generation_config_t const& config);

  [[nodiscard]] grasp_evaluation_t evaluate_grasp(
    grasp_problem_t const& problem, grasp_generation_config_t const& config,
    grasp_t const& grasp);

  [[nodiscard]] grasp_t apply_grasp_step(
    grasp_t const& grasp, Eigen::Ref<vector7_t const> step);

  [[nodiscard]] joint_evaluation_t evaluate_joint(
    joint_grasp_problem_t const& problem,
    grasp_generation_config_t const& config, Eigen::VectorXd const& variables);

  [[nodiscard]] Matrix6X body_pose_jacobian(
    KinematicSnapshot const& snapshot, LinkId link,
    pose_t const& link_from_body);

  // Shared collision/Jacobian boundary for grasp optimization and the
  // quasi-dynamic validator. The first phase is the fixed validation scene.
  [[nodiscard]] std::vector<grasp_dynamics_contact_t>
  evaluate_grasp_dynamics_contacts(
    grasp_problem_t const& problem, grasp_t const& grasp,
    grasp_dynamics_config_t const& config);

}  // namespace stacking_core::grasp_detail
