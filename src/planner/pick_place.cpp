#include <stacking_core/planner/pick_place.hpp>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace stacking_core {
  namespace {

    grasp_result_t fallback_grasps(
      phase_scene_t const& phase, direct_plan_problem_t const& problem,
      direct_plan_config_t const& config) {
      grasp_result_t sampled = sample_grasps(
        grasp_sampling_problem_t {
          .phases = {phase},
          .gripper = problem.gripper,
        },
        config.grasp_sampling, config.grasp_generation);
      if (
        sampled.status != solve_status_e::success ||
        !config.simulation_refinement) {
        return sampled;
      }

      std::vector<grasp_candidate_t> feasible;
      std::vector<grasp_candidate_t> failed;
      for (grasp_candidate_t const& candidate : sampled.candidates) {
        if (!candidate.failure.code.empty()) {
          failed.push_back(candidate);
          continue;
        }
        grasp_simulation_result_t const simulated = simulate_grasp(
          grasp_simulation_problem_t {
            .phase = phase,
            .gripper = problem.gripper,
            .initial_grasp = candidate.grasp,
          },
          config.grasp_simulation);
        if (simulated.status != solve_status_e::success) {
          grasp_candidate_t rejected = candidate;
          rejected.failure = simulated.failure;
          failed.push_back(std::move(rejected));
          continue;
        }
        grasp_result_t refined = solve_grasp_pose(
          grasp_problem_t {
            .phases = {phase},
            .gripper = problem.gripper,
            .seed = simulated.grasp,
          },
          config.grasp_generation);
        if (
          refined.status == solve_status_e::success &&
          refined.selected_candidate() != nullptr) {
          feasible.push_back(*refined.selected_candidate());
        } else if (!refined.candidates.empty()) {
          failed.push_back(std::move(refined.candidates.front()));
        }
      }
      std::size_t const feasible_count = feasible.size();
      feasible.insert(
        feasible.end(), std::make_move_iterator(failed.begin()),
        std::make_move_iterator(failed.end()));
      if (feasible_count == 0) {
        return grasp_result_t {
          .status = solve_status_e::infeasible,
          .candidates = std::move(feasible),
          .selected_index = std::nullopt,
          .failure =
            planner_failure_t {
              .code = "fallback_grasp_simulation",
              .message = "no fallback grasp passed simulation and refinement",
              .retryable = true,
            },
        };
      }
      return grasp_result_t {
        .status = solve_status_e::success,
        .candidates = std::move(feasible),
        .selected_index = std::size_t {0},
        .failure = {},
      };
    }

    std::vector<grasp_candidate_t> feasible_candidates(
      grasp_result_t const& result) {
      std::vector<grasp_candidate_t> out;
      for (grasp_candidate_t const& candidate : result.candidates) {
        if (candidate.failure.code.empty()) {
          out.push_back(candidate);
        }
      }
      return out;
    }

  }  // namespace

  plan_result_t solve_pick_place(
    pick_place_problem_t const& problem, pick_place_config_t const& config) {
    plan_result_t direct = solve_direct(problem.direct, config.direct);
    if (
      direct.status == solve_status_e::success || !config.allow_regrasp ||
      direct.status == solve_status_e::invalid_problem ||
      !direct.failure.retryable) {
      return direct;
    }

    grasp_result_t const pick = problem.pick_grasp_candidates.empty()
      ? fallback_grasps(problem.direct.pick, problem.direct, config.direct)
      : grasp_result_t {
          .status = solve_status_e::success,
          .candidates = problem.pick_grasp_candidates,
          .selected_index = std::size_t {0},
          .failure = {},
        };
    grasp_result_t const place = problem.place_grasp_candidates.empty()
      ? fallback_grasps(problem.direct.place, problem.direct, config.direct)
      : grasp_result_t {
          .status = solve_status_e::success,
          .candidates = problem.place_grasp_candidates,
          .selected_index = std::size_t {0},
          .failure = {},
        };
    if (
      pick.status != solve_status_e::success ||
      place.status != solve_status_e::success) {
      return plan_result_t {
        .status = solve_status_e::infeasible,
        .candidates = {},
        .selected_index = std::nullopt,
        .failure =
          planner_failure_t {
            .code = "fallback_grasp_generation",
            .message =
              "direct planning failed and independent fallback grasps could "
              "not be generated",
            .retryable = true,
          },
      };
    }

    plan_result_t fallback = solve_regrasp(
      regrasp_problem_t {
        .pick = problem.direct.pick,
        .handoff = problem.handoff,
        .place = problem.direct.place,
        .robot = problem.direct.robot,
        .pick_grasps = feasible_candidates(pick),
        .place_grasps = feasible_candidates(place),
        .handoff_position = problem.handoff_position,
      },
      config.regrasp);
    if (fallback.status == solve_status_e::success) {
      for (plan_candidate_t& candidate : fallback.candidates) {
        candidate.diagnostics.push_back(plan_diagnostic_t {
          .stage = std::nullopt,
          .failure = direct.failure,
        });
      }
      return fallback;
    }
    fallback.failure.message = "direct planning failed at '" +
      direct.failure.code +
      "'; regrasp fallback also failed: " + fallback.failure.message;
    return fallback;
  }

}  // namespace stacking_core
