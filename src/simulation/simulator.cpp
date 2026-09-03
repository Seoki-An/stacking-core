#include <stacking_core/simulation/simulator.hpp>

#include "constraint_solver.hpp"

#include <stacking_core/collision.hpp>
#include <stacking_core/contact.hpp>
#include <stacking_core/geometry/dsf_vert.hpp>
#include <stacking_core/simulation/integrator.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace stacking_core {
namespace {

using simulation_detail::constraint_factor_t;
using simulation_detail::constraint_node_t;
using simulation_detail::factor_id_t;
using simulation_detail::factor_kind_e;
using simulation_detail::matrix_x6_t;
using simulation_detail::vector_x_t;

void validate_config(simulation_config_t const& config) {
  if (!config.gravity.allFinite() ||
      !std::isfinite(config.contact.error_reduction_ratio) ||
      config.contact.error_reduction_ratio < 0.0 ||
      config.contact.error_reduction_ratio > 1.0 ||
      !std::isfinite(config.contact.detection_margin) ||
      config.contact.detection_margin < 0.0 ||
      !std::isfinite(config.contact.patch_eps) ||
      config.contact.patch_eps <= 0.0 ||
      !std::isfinite(config.contact.patch_damping) ||
      config.contact.patch_damping < 0.0 ||
      config.contact.projection.max_iters <= 0 ||
      !std::isfinite(config.contact.projection.tol) ||
      config.contact.projection.tol <= 0.0 ||
      !std::isfinite(config.contact.projection.friction_ratio_thresh) ||
      config.contact.projection.friction_ratio_thresh < 0.0) {
    throw std::invalid_argument("invalid simulation configuration");
  }
  for (auto const& [id, ratio] : config.contact.body_error_reduction_ratio) {
    (void)id;
    if (!std::isfinite(ratio) || ratio < 0.0 || ratio > 1.0) {
      throw std::invalid_argument(
        "invalid per-body error reduction ratio");
    }
  }
}

// diffsim's pair rule: the larger of the two bodies' ratios.
Scalar contact_error_reduction_ratio(
  simulation_contact_config_t const& config,
  EntityId first,
  EntityId second) {
  auto ratio_of = [&](EntityId id) {
    auto const found = config.body_error_reduction_ratio.find(id);
    return found == config.body_error_reduction_ratio.end()
      ? config.error_reduction_ratio
      : found->second;
  };
  return std::max(ratio_of(first), ratio_of(second));
}

Vector6 vectorized(motion_t const& motion) {
  Vector6 out;
  out << motion.linear, motion.angular;
  return out;
}

motion_t motion_from(Vector6 const& value) {
  return motion_t {
    .linear = value.head<3>(),
    .angular = value.tail<3>(),
  };
}

std::shared_ptr<SceneSnapshot const> copy_snapshot(
  SceneSnapshot const& scene) {
  std::vector<BodyInstance> bodies;
  bodies.reserve(scene.bodyCount());
  for (std::size_t i = 0; i < scene.bodyCount(); ++i) {
    BodyInstance const& body = scene.body(i);
    bodies.emplace_back(body_instance_config_t {
      .id = body.id(),
      .model = body.modelPtr(),
      .frame_from_body = body.frameFromBody(),
      .motion = body.motion(),
      .mobility = body.mobility(),
    });
  }
  return std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
    .frame = scene.frame(),
    .bodies = std::move(bodies),
  });
}

Vector6 known_motion(BodyInstance const& body) {
  if (body.mobility() == mobility_e::static_body) {
    return Vector6::Zero();
  }
  return vectorized(body.motion());
}

Matrix6 mass_matrix(BodyInstance const& body) {
  inertial_t const& inertial = body.model().inertial();
  Matrix3 const R_fb = body.frameFromBody().orientation.toRotationMatrix();
  Matrix3 const R_bi =
    inertial.body_from_inertial.orientation.toRotationMatrix();
  Matrix3 const I_body = R_bi * inertial.inertia * R_bi.transpose();
  Matrix3 const A =
    -R_fb * skew(inertial.body_from_inertial.position);

  Matrix6 M = Matrix6::Zero();
  M.topLeftCorner<3, 3>() = inertial.mass * Matrix3::Identity();
  M.topRightCorner<3, 3>() = inertial.mass * A;
  M.bottomLeftCorner<3, 3>() = inertial.mass * A.transpose();
  M.bottomRightCorner<3, 3>() =
    I_body + inertial.mass * A.transpose() * A;
  return M;
}

Vector6 gravity_force(
  BodyInstance const& body, Vector3 const& gravity) {
  inertial_t const& inertial = body.model().inertial();
  Matrix3 const R_fb = body.frameFromBody().orientation.toRotationMatrix();
  Matrix3 const A =
    -R_fb * skew(inertial.body_from_inertial.position);
  Vector3 const force = inertial.mass * gravity;
  Vector6 out;
  out << force, A.transpose() * force;
  return out;
}

Matrix36 point_velocity_jac(
  BodyInstance const& body, Vector3 const& point_frame) {
  pose_t const& frame_from_body = body.frameFromBody();
  Matrix3 const R_fb = frame_from_body.orientation.toRotationMatrix();
  Matrix36 J;
  J << Matrix3::Identity(),
    -R_fb * skew(R_fb.transpose() *
      (point_frame - frame_from_body.position));
  return J;
}

elliptical_contact_patch_t estimate_patch(
  SceneSnapshot const& scene,
  contact_t const& contact,
  simulation_contact_config_t const& config) {
  elliptical_contact_patch_t patch;
  if (contact.feature.gap >= config.patch_eps) {
    return patch;
  }

  Geometry const& geom_1 = scene.body(contact.feature.pair.first.entity)
    .model().geometry(contact.feature.pair.first.geometry);
  Geometry const& geom_2 = scene.body(contact.feature.pair.second.entity)
    .model().geometry(contact.feature.pair.second.geometry);
  bool const dsf_1 = geom_1.type() == geometry_type_e::dsf_vert;
  bool const dsf_2 = geom_2.type() == geometry_type_e::dsf_vert;
  if (!dsf_1 && !dsf_2) {
    return patch;
  }

  Eigen::Matrix<Scalar, 3, 2> const T =
    contact.frame.frameFromContact().leftCols<2>();
  auto projected_curvature = [&](Geometry const& geom,
                                 BodyInstance const& body,
                                 Vector3 const& dir) {
    auto const& dsf = static_cast<DsfVertGeometry const&>(geom);
    Matrix3 const ds_dx =
      dsf.diffable_support(dir, body.frameFromBody()).ds_dx;
    return (T.transpose() * ds_dx * T +
      config.patch_damping * Eigen::Matrix2d::Identity()).eval();
  };

  Scalar a = 0.0;
  Scalar b = 0.0;
  Scalar const depth = config.patch_eps - contact.feature.gap;
  if (dsf_1 && dsf_2) {
    Eigen::Matrix2d const H_1 = projected_curvature(
      geom_1,
      scene.body(contact.feature.pair.first.entity),
      contact.feature.normal);
    Eigen::Matrix2d const H_2 = projected_curvature(
      geom_2,
      scene.body(contact.feature.pair.second.entity),
      -contact.feature.normal);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> const eig {
      (H_1.inverse() + H_2.inverse()) / depth};
    if (eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() > 0.0) {
      a = std::sqrt(2.0 / eig.eigenvalues().minCoeff());
      b = std::sqrt(2.0 / eig.eigenvalues().maxCoeff());
    }
  } else {
    Geometry const& geom = dsf_1 ? geom_1 : geom_2;
    BodyInstance const& body = scene.body(
      dsf_1
        ? contact.feature.pair.first.entity
        : contact.feature.pair.second.entity);
    Vector3 const dir = dsf_1
      ? contact.feature.normal
      : -contact.feature.normal;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> const eig {
      projected_curvature(geom, body, dir) * depth};
    if (eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() > 0.0) {
      a = std::sqrt(2.0 * eig.eigenvalues().maxCoeff());
      b = std::sqrt(2.0 * eig.eigenvalues().minCoeff());
    }
  }
  if (std::isfinite(a) && std::isfinite(b) && a >= b && b >= 0.0) {
    patch.semi_axis_first = a;
    patch.semi_axis_second = b;
  }
  return patch;
}

constraint_factor_t make_contact_factor(
  SceneSnapshot const& scene,
  contact_t const& contact,
  simulation_config_t const& config,
  Scalar dt) {
  BodyInstance const& body_1 =
    scene.body(contact.feature.pair.first.entity);
  BodyInstance const& body_2 =
    scene.body(contact.feature.pair.second.entity);
  Matrix3 const R_cf = contact.frame.frameFromContact().transpose();
  Matrix36 const J_point_1 =
    point_velocity_jac(body_1, contact.feature.point_first);
  Matrix36 const J_point_2 =
    point_velocity_jac(body_2, contact.feature.point_second);

  bool const use_4d = config.contact.model ==
    simulation_contact_model_e::limit_surface_4d;
  Eigen::Index const dim = use_4d ? 4 : 3;
  matrix_x6_t C_1(dim, 6);
  matrix_x6_t C_2(dim, 6);
  C_1.topRows<3>() = R_cf * J_point_1;
  C_2.topRows<3>() = R_cf * J_point_2;
  if (use_4d) {
    C_1.bottomRows<1>().setZero();
    C_2.bottomRows<1>().setZero();
    C_1.block<1, 3>(3, 3) =
      -contact.feature.normal.transpose() *
      body_1.frameFromBody().orientation.toRotationMatrix();
    C_2.block<1, 3>(3, 3) =
      -contact.feature.normal.transpose() *
      body_2.frameFromBody().orientation.toRotationMatrix();
  }

  vector_x_t error = vector_x_t::Zero(dim);
  error[2] = contact_error_reduction_ratio(
    config.contact, body_1.id(), body_2.id()) * contact.feature.gap;
  constraint_factor_t factor {
    .id = factor_id_t {
      .kind = use_4d
        ? factor_kind_e::contact_4d
        : factor_kind_e::contact_3d,
      .first = body_1.id(),
      .second = body_2.id(),
      .first_geometry = contact.feature.pair.first.geometry,
      .second_geometry = contact.feature.pair.second.geometry,
    },
    .hard_inequality = true,
    .entities = {},
    .jacobians = {},
    .error = std::move(error),
    .projector = {},
  };

  matrix_x6_t const J_1 = -dt * C_1;
  matrix_x6_t const J_2 = dt * C_2;
  if (body_1.isDynamic()) {
    factor.entities.push_back(body_1.id());
    factor.jacobians.push_back(J_1);
  } else {
    factor.error += J_1 * known_motion(body_1);
  }
  if (body_2.isDynamic()) {
    factor.entities.push_back(body_2.id());
    factor.jacobians.push_back(J_2);
  } else {
    factor.error += J_2 * known_motion(body_2);
  }

  if (use_4d) {
    elliptical_contact_patch_t const patch =
      estimate_patch(scene, contact, config.contact);
    Scalar torsional_friction =
      torsional_friction_coefficient(contact.friction, patch);
    if (contact.friction > 0.0) {
      torsional_friction = std::max(
        torsional_friction, std::numeric_limits<Scalar>::epsilon());
    }
    factor.projector =
      [friction = contact.friction,
       torsional_friction,
       projection = config.contact.projection](vector_x_t const& v) {
        Vector4 const projected = project_limit_surface_impulse(
          v.head<4>(), friction, torsional_friction, projection);
        return vector_x_t {projected};
      };
  } else {
    factor.projector = [friction = contact.friction](vector_x_t const& v) {
      Vector3 const projected = project_coulomb_impulse(v.head<3>(), friction);
      return vector_x_t {projected};
    };
  }
  return factor;
}

}  // namespace

class Simulator::Impl {
public:
  explicit Impl(simulation_config_t config): config_(std::move(config)) {
    validate_config(config_);
  }

  simulation_result_t step(SceneSnapshot const& scene, Scalar dt) {
    if (!std::isfinite(dt) || dt <= 0.0) {
      throw std::invalid_argument("simulation dt must be finite and positive");
    }

    solver_.clear_factors();
    // The equations of motion are the objective of the constraint solve, so
    // they are handed to it as per-body dynamics rather than as factors.
    std::map<EntityId, constraint_node_t> nodes;
    Scalar total_mass = 0.0;
    int dynamic_count = 0;
    for (std::size_t i = 0; i < scene.bodyCount(); ++i) {
      BodyInstance const& body = scene.body(i);
      if (!body.isDynamic()) {
        continue;
      }
      if (!body.model().hasInertial()) {
        throw std::invalid_argument("dynamic body requires inertial data");
      }
      inertial_t const& inertial = body.model().inertial();
      total_mass += inertial.mass;
      ++dynamic_count;

      Matrix6 const M = mass_matrix(body);
      Eigen::LLT<Matrix6> const M_llt {M};
      if (M_llt.info() != Eigen::Success) {
        throw std::runtime_error("dynamic body mass matrix is not positive definite");
      }
      Vector6 const velocity = vectorized(body.motion());
      nodes.emplace(body.id(), constraint_node_t {
        .mass = M,
        .momentum = M * velocity + gravity_force(body, config_.gravity) * dt,
        .velocity = velocity,
      });
    }

    std::vector<EntityId> entities;
    entities.reserve(scene.bodyCount());
    for (std::size_t i = 0; i < scene.bodyCount(); ++i) {
      entities.push_back(scene.body(i).id());
    }
    std::shared_ptr<SceneSnapshot const> const scene_ref(
      &scene, [](SceneSnapshot const*) {});
    SceneView const view {scene_ref, std::move(entities)};
    std::vector<collision_body_pair_t> const body_pairs =
      bounding_volume_broad_phase(view, config_.contact.detection_margin);
    std::vector<collision_pair_t> const geom_pairs =
      bounding_volume_middle_phase(
        view, body_pairs, config_.contact.detection_margin);

    std::vector<contact_t> contacts;
    for (collision_pair_t const& pair : geom_pairs) {
      BodyInstance const& body_1 = scene.body(pair.first.entity);
      BodyInstance const& body_2 = scene.body(pair.second.entity);
      if (!body_1.isDynamic() && !body_2.isDynamic()) {
        continue;
      }
      contact_feature_t feature = compute_contact(scene, pair);
      if (feature.gap > config_.contact.detection_margin) {
        continue;
      }
      contact_t contact = make_contact(scene, std::move(feature));
      solver_.add_factor(make_contact_factor(scene, contact, config_, dt));
      contacts.push_back(std::move(contact));
    }

    Scalar const dynamics_scale = dynamic_count > 0
      ? std::max(Scalar {1.0}, total_mass / dynamic_count)
      : Scalar {1.0};
    simulation_solver_stats_t const stats =
      solver_.solve(nodes, config_.solver, dynamics_scale);

    std::vector<BodyInstance> bodies;
    bodies.reserve(scene.bodyCount());
    for (std::size_t i = 0; i < scene.bodyCount(); ++i) {
      BodyInstance const& body = scene.body(i);
      motion_t motion = body.motion();
      if (body.isDynamic()) {
        motion = motion_from(nodes.at(body.id()).velocity);
      }
      pose_t pose = body.frameFromBody();
      if (body.isMovable()) {
        pose = integrate_pose(pose, motion, dt);
      }
      bodies.emplace_back(body_instance_config_t {
        .id = body.id(),
        .model = body.modelPtr(),
        .frame_from_body = std::move(pose),
        .motion = std::move(motion),
        .mobility = body.mobility(),
      });
    }
    auto snapshot = std::make_shared<SceneSnapshot>(scene_snapshot_config_t {
      .frame = scene.frame(),
      .bodies = std::move(bodies),
    });
    return simulation_result_t {
      .snapshot = std::move(snapshot),
      .contacts = std::move(contacts),
      .solver = stats,
    };
  }

  simulation_result_t step_n(
    SceneSnapshot const& scene, Scalar dt, std::size_t count) {
    if (count == 0) {
      return simulation_result_t {
        .snapshot = copy_snapshot(scene),
        .contacts = {},
        .solver = {},
      };
    }

    simulation_result_t result = step(scene, dt);
    for (std::size_t iter = 1; iter < count; ++iter) {
      result = step(*result.snapshot, dt);
    }
    return result;
  }

  simulation_config_t config_;
  simulation_detail::ConstraintSolver solver_;
};

Simulator::Simulator(simulation_config_t config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
}

Simulator::~Simulator() = default;
Simulator::Simulator(Simulator&&) noexcept = default;
Simulator& Simulator::operator=(Simulator&&) noexcept = default;

simulation_config_t const& Simulator::config() const noexcept {
  return impl_->config_;
}

simulation_result_t Simulator::step(SceneSnapshot const& scene, Scalar dt) {
  return impl_->step(scene, dt);
}

simulation_result_t Simulator::step_n(
  SceneSnapshot const& scene, Scalar dt, std::size_t count) {
  return impl_->step_n(scene, dt, count);
}

void Simulator::clearWarmStart() {
  impl_->solver_.clear_warm_start();
}

}  // namespace stacking_core
