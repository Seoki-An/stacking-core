#pragma once

#include <stacking_core/types.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace stacking_core {

// Approximately solves H * v = b inside ||v|| <= delta.
// Negative curvature and boundary crossings return the first intersection with
// the trust-region boundary, as required by the Steihaug method.
template <typename VectorType, typename MatrixType>
[[nodiscard]] std::pair<typename VectorType::PlainObject, bool>
truncated_conjugate_gradient(
  VectorType const& b,
  MatrixType const& H,
  Scalar tol,
  Scalar delta) {
  using plain_vector_t = typename VectorType::PlainObject;
  if (!b.allFinite() || !H.allFinite()) {
    throw std::invalid_argument("trust-region inputs must be finite");
  }
  if (!std::isfinite(tol) || tol <= 0.0 ||
      !std::isfinite(delta) || delta <= 0.0) {
    throw std::invalid_argument(
      "trust-region tolerances must be finite and positive");
  }

  plain_vector_t v = plain_vector_t::Zero(b.rows());
  plain_vector_t r = b;
  plain_vector_t p = r;
  Scalar const r_init_norm = r.norm();
  if (r_init_norm == 0.0) {
    return {v, false};
  }

  auto boundary_step = [&](plain_vector_t const& v_cur,
                           plain_vector_t const& p_cur) {
    Scalar const v_p = v_cur.dot(p_cur);
    Scalar const p_sq = p_cur.squaredNorm();
    Scalar const v_sq = v_cur.squaredNorm();
    Scalar const discr = std::max(
      Scalar {0.0},
      v_p * v_p + p_sq * (delta * delta - v_sq));
    Scalar const t = (-v_p + std::sqrt(discr)) / p_sq;
    return (v_cur + t * p_cur).eval();
  };

  int const max_iters = static_cast<int>(b.rows()) + 50;
  for (int iter = 0; iter < max_iters; ++iter) {
    plain_vector_t const H_p = H * p;
    Scalar const p_H_p = p.dot(H_p);
    if (p_H_p <= 0.0) {
      return {boundary_step(v, p), true};
    }

    Scalar const r_sq = r.squaredNorm();
    Scalar const a = r_sq / p_H_p;
    plain_vector_t const v_new = v + a * p;
    if (v_new.norm() > delta) {
      return {boundary_step(v, p), true};
    }

    plain_vector_t const r_new = r - a * H_p;
    if (r_new.norm() < r_init_norm * std::min(r_init_norm, tol)) {
      return {v_new, false};
    }
    p = r_new + r_new.squaredNorm() / r_sq * p;
    v = v_new;
    r = r_new;
  }
  return {v, false};
}

}  // namespace stacking_core
