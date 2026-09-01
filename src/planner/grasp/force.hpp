#pragma once

#include "evaluation.hpp"

#include <stacking_core/collision.hpp>

#include <cstddef>
#include <vector>

namespace stacking_core::grasp_detail {

  struct force_contact_t {
    diffable_contact_feature_t feature;
    matrix67_t body_jac = matrix67_t::Zero();
    std::size_t contact_index = 0;
  };

  struct force_evaluation_t {
    Scalar cost = 0.0;
    Eigen::VectorXd grad;
    Eigen::MatrixXd hess;

    [[nodiscard]] bool finite() const noexcept;
  };

  struct force_refinement_t {
    Scalar cost = 0.0;
    vector7_t grad = vector7_t::Zero();
    std::vector<Vector3> contact_forces;

    [[nodiscard]] bool finite() const noexcept;
  };

  [[nodiscard]] force_evaluation_t evaluate_contact_forces(
    Eigen::Ref<Eigen::VectorXd const> forces,
    std::vector<force_contact_t> const& contacts, Vector6 const& wrench,
    Vector3 const& center, grasp_force_config_t const& config);

  [[nodiscard]] force_refinement_t refine_contact_forces(
    std::vector<force_contact_t> const& contacts, Vector3 const& center,
    grasp_force_config_t const& config);

}  // namespace stacking_core::grasp_detail
