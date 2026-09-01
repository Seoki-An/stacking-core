#include <stacking_core/posegen/pose_generator.hpp>

#include "objective.hpp"

#include <stacking_core/geometry/dsf_vert.hpp>
#include <stacking_core/optimization/truncated_conjugate_gradient.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace stacking_core {
namespace {

Quaternion exp_rotation(Vector3 const& rotation) {
  Scalar const angle = rotation.norm();
  if (angle <= std::numeric_limits<Scalar>::epsilon()) {
    return Quaternion::Identity();
  }
  return Quaternion {Eigen::AngleAxis<Scalar> {angle, rotation / angle}};
}

bool has_dsf(BodyInstance const& body) {
  for (std::size_t idx = 0; idx < body.model().geometryCount(); ++idx) {
    if (body.model().geometry(idx).type() == geometry_type_e::dsf_vert) {
      return true;
    }
  }
  return false;
}

void validate_config(posegen_config_t const& config) {
  auto finite = [](Scalar value) { return std::isfinite(value); };
  auto const& tr = config.trust_region;
  auto const& hausdorff = config.hausdorff;
  auto const& obj = config.objective;
  auto const& force = config.force_solver;
  if (tr.max_iters <= 0 || !finite(tr.eps) || tr.eps <= 0.0 ||
      !finite(tr.delta_init) || tr.delta_init <= 0.0 ||
      !finite(tr.delta_max) || tr.delta_max <= 0.0 ||
      tr.delta_init > tr.delta_max ||
      !finite(tr.delta_reduction_rate) || tr.delta_reduction_rate <= 0.0 ||
      tr.delta_reduction_rate >= 1.0 ||
      !finite(tr.delta_expansion_rate) || tr.delta_expansion_rate <= 1.0 ||
      !finite(tr.delta_lower_thresh) || !finite(tr.delta_upper_thresh) ||
      !finite(tr.improvement_thresh) || !finite(tr.tol) || tr.tol <= 0.0 ||
      hausdorff.max_iters <= 0 || !finite(hausdorff.eps) ||
      hausdorff.eps <= 0.0 || !finite(hausdorff.error_tol) ||
      hausdorff.error_tol <= 0.0 || !finite(hausdorff.radius_init) ||
      hausdorff.radius_init <= 0.0 ||
      !finite(hausdorff.radius_reduction_rate) ||
      hausdorff.radius_reduction_rate <= 0.0 ||
      hausdorff.radius_reduction_rate >= 1.0 ||
      !finite(hausdorff.radius_expansion_rate) ||
      hausdorff.radius_expansion_rate <= 1.0 ||
      !obj.k_wrench.allFinite() || !finite(obj.rho) || obj.rho <= 0.0 ||
      !finite(obj.narrow_phase_scene_tol) ||
      obj.narrow_phase_scene_tol < 0.0 ||
      !finite(obj.narrow_phase_candidate_tol) ||
      obj.narrow_phase_candidate_tol < 0.0 ||
      !finite(obj.eps_gap) || obj.eps_gap < 0.0 ||
      !finite(obj.eps_comp) || obj.eps_comp < 0.0 ||
      !finite(obj.eps_cone) || obj.eps_cone <= 0.0 ||
      !finite(obj.eps_target) || obj.eps_target < 0.0 ||
      !finite(obj.k_comp) || obj.k_comp < 0.0 ||
      !finite(obj.k_gap) || obj.k_gap < 0.0 ||
      !finite(obj.k_gap_c) || obj.k_gap_c < 0.0 ||
      !finite(obj.k_target) || obj.k_target < 0.0 ||
      !finite(obj.k_potential) || obj.k_potential < 0.0 ||
      !finite(obj.k_xy) || obj.k_xy < 0.0 ||
      !finite(obj.k_box) || obj.k_box < 0.0 ||
      !finite(obj.k_reg) || obj.k_reg < 0.0 ||
      !finite(obj.k_lower) || obj.k_lower < 0.0 ||
      !finite(obj.w_box) || obj.w_box <= 0.0 ||
      !finite(obj.gravity) || obj.gravity < 0.0 ||
      !finite(obj.ground_height) || force.max_iters <= 0 ||
      !finite(force.beta_consensus) || force.beta_consensus <= 0.0 ||
      !finite(force.beta_contact) || force.beta_contact <= 0.0 ||
      force.beta_update_interval <= 0 || !finite(force.tol_abs) ||
      force.tol_abs <= 0.0 || !finite(force.tol_rel) ||
      force.tol_rel < 0.0) {
    throw std::invalid_argument("invalid posegen configuration");
  }
}

void validate_problem(posegen_problem_t const& problem) {
  if (!problem.candidate.valid() || !problem.scene.contains(problem.candidate)) {
    throw std::invalid_argument("posegen candidate must belong to the scene view");
  }
  BodyInstance const& candidate = problem.scene.body(problem.candidate);
  if (!has_dsf(candidate)) {
    throw std::invalid_argument("posegen candidate requires DSF-Vert geometry");
  }
  if (candidate.model().hasInertial()) {
    Scalar const mass = candidate.model().inertial().mass;
    if (!std::isfinite(mass) || mass <= 0.0) {
      throw std::invalid_argument("posegen candidate mass must be positive");
    }
  }

  std::set<EntityId> targets;
  for (EntityId id : problem.targets) {
    if (id == problem.candidate || !problem.scene.contains(id) ||
        !targets.insert(id).second) {
      throw std::invalid_argument("invalid posegen target role");
    }
    if (!has_dsf(problem.scene.body(id))) {
      throw std::invalid_argument("posegen target requires DSF-Vert geometry");
    }
  }
  std::set<EntityId> boundaries;
  for (EntityId id : problem.boundaries) {
    if (id == problem.candidate || targets.contains(id) ||
        !problem.scene.contains(id) || !boundaries.insert(id).second) {
      throw std::invalid_argument("invalid posegen boundary role");
    }
  }
}

}  // namespace

class PoseGenerator::Impl {
public:
  explicit Impl(posegen_config_t config): config_(std::move(config)) {
    validate_config(config_);
  }

  void set_config(posegen_config_t config) {
    validate_config(config);
    bool const invalidates_scene_contacts =
      config.objective.ground_height != config_.objective.ground_height ||
      config.objective.narrow_phase_scene_tol !=
        config_.objective.narrow_phase_scene_tol;
    config_ = std::move(config);
    if (invalidates_scene_contacts) {
      objective_.reset();
    }
  }

  posegen_result_t solve(posegen_problem_t const& problem) {
    validate_problem(problem);
    if (!objective_ || !objective_->can_reuse(problem)) {
      objective_ =
        std::make_unique<posegen_detail::PoseObjective>(config_, problem);
    } else {
      objective_->begin_solve(problem);
    }
    posegen_detail::PoseObjective& objective = *objective_;
    pose_t pose = problem.scene.body(problem.candidate).frameFromBody();
    posegen_detail::objective_eval_t current = objective.eval(pose);
    Matrix6 hess = current.hess;
    Scalar delta = config_.trust_region.delta_init;
    posegen_solver_stats_t solver;
    solver.objective_evals = 1;
    solver.converged = false;
    solver.scene_graph_rebuilds = current.scene_graph_rebuilds;
    solver.scene_graph_reuses = current.scene_graph_reuses;

    for (int iter = 0; iter < config_.trust_region.max_iters; ++iter) {
      if (current.grad.norm() < config_.trust_region.tol ||
          delta < config_.trust_region.tol) {
        solver.converged = true;
        break;
      }
      solver.iters = iter + 1;
      auto const [step, boundary] = truncated_conjugate_gradient(
        -current.grad, hess, config_.trust_region.eps, delta);
      pose_t next_pose {
        pose.position + step.head<3>(),
        pose.orientation * exp_rotation(step.tail<3>()),
      };
      posegen_detail::objective_eval_t next = objective.eval(next_pose);
      ++solver.objective_evals;
      solver.scene_graph_rebuilds = next.scene_graph_rebuilds;
      solver.scene_graph_reuses = next.scene_graph_reuses;
      Scalar const model = current.objective + current.grad.dot(step) +
        0.5 * step.dot(hess * step);
      Scalar const rho =
        (current.objective - next.objective) /
        (current.objective - model);
      if (rho > config_.trust_region.improvement_thresh || std::isnan(rho)) {
        Matrix6 transport = Matrix6::Identity();
        transport.topLeftCorner<3, 3>() =
          exp_rotation(-0.5 * step.tail<3>()).toRotationMatrix();
        Matrix6 const transport_inv = transport.transpose();
        Vector6 const y = transport_inv * next.grad - current.grad;
        Vector6 const transported_y = transport * y;
        if (std::abs(y.dot(step)) < 1e-8) {
          hess = current.hess;
        } else {
          Vector6 const transported_hess_step = transport * hess * step;
          hess = transport * hess * transport_inv +
            transported_y * transported_y.transpose() / y.dot(step) -
            transported_hess_step * transported_hess_step.transpose() /
              step.dot(hess * step);
        }
        pose = std::move(next_pose);
        current = std::move(next);
        ++solver.accepted_iters;
      }
      if (rho < config_.trust_region.delta_lower_thresh) {
        delta *= config_.trust_region.delta_reduction_rate;
      } else if (rho > config_.trust_region.delta_upper_thresh && boundary) {
        delta = std::min(
          config_.trust_region.delta_expansion_rate * delta,
          config_.trust_region.delta_max);
      }
    }
    if (current.grad.norm() < config_.trust_region.tol ||
        delta < config_.trust_region.tol) {
      solver.converged = true;
    }
    solver.grad_norm = current.grad.norm();
    solver.trust_region_radius = delta;
    solver.force_solver = current.force_solver;

    std::vector<Vector3> const& candidate_forces =
      current.contact_force[problem.candidate];
    Matrix3X force_matrix(3, static_cast<Eigen::Index>(candidate_forces.size()));
    for (std::size_t idx = 0; idx < candidate_forces.size(); ++idx) {
      force_matrix.col(static_cast<Eigen::Index>(idx)) = candidate_forces[idx];
    }
    return posegen_result_t {
      .optimal_pose = std::move(pose),
      .candidate_contact_forces = std::move(force_matrix),
      .c_feq = current.c_feq,
      .c_comp = current.c_comp,
      .c_gap = current.c_gap,
      .net_wrench = std::move(current.net_wrench),
      .contact_force = std::move(current.contact_force),
      .contact_point = std::move(current.contact_point),
      .contact_normal = std::move(current.contact_normal),
      .solver = solver,
    };
  }

  posegen_config_t config_;
  std::unique_ptr<posegen_detail::PoseObjective> objective_;
};

PoseGenerator::PoseGenerator(posegen_config_t config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
}

PoseGenerator::~PoseGenerator() = default;
PoseGenerator::PoseGenerator(PoseGenerator&&) noexcept = default;
PoseGenerator& PoseGenerator::operator=(PoseGenerator&&) noexcept = default;

posegen_config_t const& PoseGenerator::config() const noexcept {
  return impl_->config_;
}

void PoseGenerator::setConfig(posegen_config_t config) {
  impl_->set_config(std::move(config));
}

posegen_result_t PoseGenerator::solve(posegen_problem_t const& problem) {
  return impl_->solve(problem);
}

}  // namespace stacking_core
