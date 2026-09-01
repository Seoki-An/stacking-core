#pragma once

#include <stacking_core/planner/manipulation.hpp>
#include <stacking_core/planner/regrasp.hpp>

#include <vector>

namespace stacking_core {

  struct pick_place_problem_t {
    direct_plan_problem_t direct;
    phase_scene_t handoff;
    Vector3 handoff_position = Vector3::Zero();

    // Optional application-generated fallback sets. If either side is empty,
    // the fallback policy samples that side independently.
    std::vector<grasp_candidate_t> pick_grasp_candidates;
    std::vector<grasp_candidate_t> place_grasp_candidates;
  };

  struct pick_place_config_t {
    direct_plan_config_t direct;
    regrasp_config_t regrasp;
    bool allow_regrasp = true;
  };

  // Attempts direct planning first. A retryable direct failure automatically
  // triggers independent pick/place grasp generation and regrasp planning.
  // The direct failure is retained in every fallback candidate diagnostic.
  [[nodiscard]] plan_result_t solve_pick_place(
    pick_place_problem_t const& problem,
    pick_place_config_t const& config = {});

}  // namespace stacking_core
