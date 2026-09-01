#include <stacking_core/planner/grasp.hpp>
#include <stacking_core/planner/manipulation.hpp>

namespace stacking_core {

  grasp_candidate_t const* grasp_result_t::selected_candidate() const noexcept {
    if (!selected_index || *selected_index >= candidates.size()) {
      return nullptr;
    }
    return &candidates[*selected_index];
  }

  joint_grasp_candidate_t const* joint_grasp_result_t::selected_candidate()
    const noexcept {
    if (!selected_index || *selected_index >= candidates.size()) {
      return nullptr;
    }
    return &candidates[*selected_index];
  }

  plan_candidate_t const* plan_result_t::selected_candidate() const noexcept {
    if (!selected_index || *selected_index >= candidates.size()) {
      return nullptr;
    }
    return &candidates[*selected_index];
  }

}  // namespace stacking_core
