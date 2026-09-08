#pragma once

#include <stacking_core/types.hpp>

#include <algorithm>
#include <map>
#include <utility>

namespace stacking_core::planner_detail {

  struct collision_key_t {
    int sample;
    EntityId first;
    EntityId second;

    auto operator<=>(collision_key_t const&) const = default;
  };

  struct collision_alm_eval_t {
    Scalar cost;
    Scalar d_violation;
  };

  // Powell-Hestenes-Rockafellar inequality term, c = required_gap - gap.
  // Unlike the clearance penalty, legacy does NOT multiply this term by dt.
  inline collision_alm_eval_t collision_alm_term(
    Scalar lambda, Scalar beta, Scalar c) {
    Scalar const active = std::max(Scalar {0.0}, lambda + beta * c);
    return {(active * active - lambda * lambda) / (2.0 * beta), active};
  }

  struct collision_alm_state_t {
    Scalar beta;
    std::map<collision_key_t, Scalar> duals;

    [[nodiscard]] Scalar multiplier(collision_key_t const& key) const {
      auto const it = duals.find(key);
      return it == duals.end() ? 0.0 : it->second;
    }

    // Replace, rather than accumulate, the recorded set so disappearing
    // contacts cannot retain stale multipliers. Called only on an accepted
    // path after the inner solve, never from a line-search evaluation.
    void update(std::map<collision_key_t, Scalar> const& violations) {
      std::map<collision_key_t, Scalar> next;
      for (auto const& [key, c] : violations) {
        next.emplace(key, std::max(Scalar {0.0}, multiplier(key) + beta * c));
      }
      duals = std::move(next);
    }
  };

}  // namespace stacking_core::planner_detail
