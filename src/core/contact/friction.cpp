#include <stacking_core/contact/friction.hpp>

#include <cmath>
#include <stdexcept>

namespace stacking_core {

Vector3 project_coulomb_impulse(
  Vector3 const& impulse_contact, Scalar friction) {
  if (!impulse_contact.allFinite()) {
    throw std::invalid_argument("contact impulse must be finite");
  }
  if (!std::isfinite(friction) || friction < 0.0) {
    throw std::invalid_argument("friction must be finite and non-negative");
  }

  Scalar const normal = impulse_contact.z();
  if (normal <= 0.0) {
    return Vector3::Zero();
  }
  Scalar const tangent_norm = impulse_contact.head<2>().norm();
  Scalar const tangent_limit = friction * normal;
  if (tangent_norm <= tangent_limit) {
    return impulse_contact;
  }

  Vector3 result = Vector3::Zero();
  if (tangent_norm > 0.0) {
    result.head<2>() =
      tangent_limit * impulse_contact.head<2>() / tangent_norm;
  }
  result.z() = normal;
  return result;
}

}  // namespace stacking_core
