#include <stacking_core/contact/limit_surface_4d.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace stacking_core {
namespace {

void validate_coefficient(Scalar value, char const* name) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(
      std::string {name} + " must be finite and non-negative");
  }
}

Vector4 project_degenerate(
  Vector4 const& impulse, Scalar friction, Scalar torsional_friction) {
  Vector4 result = Vector4::Zero();
  Scalar const normal = impulse.z();
  result.z() = normal;

  if (friction > 0.0) {
    Scalar const tangent_norm = impulse.head<2>().norm();
    Scalar const tangent_limit = friction * normal;
    if (tangent_norm <= tangent_limit) {
      result.head<2>() = impulse.head<2>();
    } else if (tangent_norm > 0.0) {
      result.head<2>() = tangent_limit * impulse.head<2>() / tangent_norm;
    }
  }
  if (torsional_friction > 0.0) {
    Scalar const torsional_limit = torsional_friction * normal;
    result.w() = std::clamp(
      impulse.w(), -torsional_limit, torsional_limit);
  }
  return result;
}

}  // namespace

Scalar torsional_friction_coefficient(
  Scalar friction, elliptical_contact_patch_t const& patch) {
  validate_coefficient(friction, "friction");
  validate_coefficient(patch.semi_axis_first, "contact patch semi-axis");
  validate_coefficient(patch.semi_axis_second, "contact patch semi-axis");

  Scalar const major =
    std::max(patch.semi_axis_first, patch.semi_axis_second);
  Scalar const minor =
    std::min(patch.semi_axis_first, patch.semi_axis_second);
  if (friction == 0.0 || major == 0.0) {
    return 0.0;
  }

  Scalar const ratio = minor / major;
  Scalar const eccentricity =
    std::sqrt(std::max(Scalar {0.0}, 1.0 - ratio * ratio));
  return friction * (4.0 * major) / (3.0 * std::numbers::pi) *
    std::comp_ellint_2(eccentricity);
}

Vector4 project_limit_surface_impulse(
  Vector4 const& impulse_contact,
  Scalar friction,
  Scalar torsional_friction,
  limit_surface_projection_config_t const& config) {
  if (!impulse_contact.allFinite()) {
    throw std::invalid_argument("contact impulse must be finite");
  }
  validate_coefficient(friction, "friction");
  validate_coefficient(torsional_friction, "torsional friction");
  if (config.max_iters <= 0 || !std::isfinite(config.tol) ||
      config.tol <= 0.0 || !std::isfinite(config.friction_ratio_thresh) ||
      config.friction_ratio_thresh < 0.0) {
    throw std::invalid_argument("invalid limit-surface projection configuration");
  }

  Scalar const normal = impulse_contact.z();
  if (normal <= 0.0) {
    return Vector4::Zero();
  }
  if (friction == 0.0 || torsional_friction == 0.0) {
    return project_degenerate(
      impulse_contact, friction, torsional_friction);
  }

  Scalar const tangent_norm = impulse_contact.head<2>().norm();
  Scalar const scaled_tangent = tangent_norm / friction;
  Scalar const scaled_torsion =
    std::abs(impulse_contact.w()) / torsional_friction;
  if (std::hypot(scaled_tangent, scaled_torsion) <= normal) {
    return impulse_contact;
  }

  if (scaled_torsion == 0.0) {
    Vector4 result = Vector4::Zero();
    result.head<2>() =
      friction * normal * impulse_contact.head<2>() / tangent_norm;
    result.z() = normal;
    return result;
  }
  if (scaled_tangent == 0.0) {
    Vector4 result = Vector4::Zero();
    result.z() = normal;
    result.w() = std::copysign(
      torsional_friction * normal, impulse_contact.w());
    return result;
  }

  Scalar angle = 0.0;
  if (torsional_friction < friction * config.friction_ratio_thresh) {
    angle = scaled_tangent > normal
      ? 0.0
      : std::acos(scaled_tangent / normal);
  } else if (friction <
             torsional_friction * config.friction_ratio_thresh) {
    angle = scaled_torsion > normal
      ? std::numbers::pi / 2.0
      : std::asin(scaled_torsion / normal);
  } else {
    Scalar const coeff_tangent =
      friction * friction * scaled_tangent;
    Scalar const coeff_torsion =
      torsional_friction * torsional_friction * scaled_torsion;
    Scalar const coeff_normal =
      (friction * friction - torsional_friction * torsional_friction) * normal;

    Scalar lower = 0.0;
    Scalar upper = std::numbers::pi / 2.0;
    angle = std::numbers::pi / 4.0;
    for (int iter = 0; iter < config.max_iters; ++iter) {
      Scalar const sine = std::sin(angle);
      Scalar const cosine = std::cos(angle);
      Scalar const res =
        coeff_tangent * sine - coeff_torsion * cosine -
        coeff_normal * sine * cosine;
      if (res > 0.0) {
        upper = angle;
      } else {
        lower = angle;
      }

      Scalar const deriv =
        coeff_tangent * cosine + coeff_torsion * sine -
        coeff_normal * (cosine * cosine - sine * sine);
      Scalar next = 0.5 * (lower + upper);
      if (deriv > 0.0) {
        Scalar const newton = angle - res / deriv;
        if (std::isfinite(newton) && newton > lower && newton < upper) {
          next = newton;
        }
      }
      bool const converged = std::abs(next - angle) <= config.tol;
      angle = next;
      if (converged) {
        break;
      }
    }
  }

  Vector4 result = Vector4::Zero();
  result.head<2>() = friction * normal * std::cos(angle) *
    impulse_contact.head<2>() / tangent_norm;
  result.z() = normal;
  result.w() = std::copysign(
    torsional_friction * normal * std::sin(angle), impulse_contact.w());
  return result;
}

}  // namespace stacking_core
