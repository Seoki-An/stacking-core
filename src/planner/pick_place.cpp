#include <stacking_core/planner/pick_place.hpp>

#include "parallel.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace stacking_core {
  namespace {

    using planner_clock_t = std::chrono::steady_clock;

    Scalar elapsed_seconds(planner_clock_t::time_point start) {
      return std::chrono::duration<Scalar>(planner_clock_t::now() - start)
        .count();
    }

    void accumulate_timings(
      planner_timings_t& out, planner_timings_t const& value) {
      out.grasp_generation_seconds += value.grasp_generation_seconds;
      out.simulation_refinement_seconds += value.simulation_refinement_seconds;
      out.trajectory_optimization_seconds +=
        value.trajectory_optimization_seconds;
      out.total_seconds += value.total_seconds;
      out.grasp_candidates += value.grasp_candidates;
      out.refined_candidates += value.refined_candidates;
      out.motion_candidates += value.motion_candidates;
    }

    struct fallback_evaluation_t {
      bool feasible = false;
      std::optional<grasp_candidate_t> candidate;
    };

    grasp_result_t fallback_grasps(
      phase_scene_t const& phase, direct_plan_problem_t const& problem,
      direct_plan_config_t const& config, planner_timings_t& timings) {
      planner_clock_t::time_point const total_start = planner_clock_t::now();
      grasp_sampling_config_t sampling_config = config.grasp_sampling;
      sampling_config.worker_count = config.worker_count;
      planner_clock_t::time_point const generation_start =
        planner_clock_t::now();
      grasp_result_t sampled = sample_grasps(
        grasp_sampling_problem_t {
          .phases = {phase},
          .gripper = problem.gripper,
        },
        sampling_config, config.grasp_generation);
      timings.grasp_generation_seconds += elapsed_seconds(generation_start);
      timings.grasp_candidates += sampled.candidates.size();
      if (
        sampled.status != solve_status_e::success ||
        !config.simulation_refinement) {
        timings.total_seconds += elapsed_seconds(total_start);
        return sampled;
      }

      planner_clock_t::time_point const refinement_start =
        planner_clock_t::now();
      std::vector<std::optional<fallback_evaluation_t>> evaluated(
        sampled.candidates.size());
      std::atomic<std::size_t> feasible_count {0};
      auto enough_candidates = [&]() {
        return feasible_count.load(std::memory_order_relaxed) >=
          static_cast<std::size_t>(config.max_candidates);
      };
      detail::planner_parallel_for(
        sampled.candidates.size(), config.worker_count, enough_candidates,
        [&](std::size_t i) {
          grasp_candidate_t const& candidate = sampled.candidates[i];
          if (!candidate.failure.code.empty()) {
            evaluated[i] = fallback_evaluation_t {
              .feasible = false,
              .candidate = candidate,
            };
            return;
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
            evaluated[i] = fallback_evaluation_t {
              .feasible = false,
              .candidate = std::move(rejected),
            };
            return;
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
            evaluated[i] = fallback_evaluation_t {
              .feasible = true,
              .candidate = *refined.selected_candidate(),
            };
            feasible_count.fetch_add(1, std::memory_order_relaxed);
          } else if (!refined.candidates.empty()) {
            evaluated[i] = fallback_evaluation_t {
              .feasible = false,
              .candidate = std::move(refined.candidates.front()),
            };
          }
        });

      std::vector<grasp_candidate_t> feasible;
      std::vector<grasp_candidate_t> failed;
      for (std::optional<fallback_evaluation_t>& value : evaluated) {
        if (!value.has_value() || !value->candidate.has_value()) {
          continue;
        }
        if (value->feasible) {
          feasible.push_back(std::move(*value->candidate));
        } else {
          failed.push_back(std::move(*value->candidate));
        }
      }
      std::stable_sort(
        feasible.begin(), feasible.end(),
        [](grasp_candidate_t const& lhs, grasp_candidate_t const& rhs) {
          return lhs.score > rhs.score;
        });
      std::size_t const accepted_count = feasible.size();
      timings.simulation_refinement_seconds +=
        elapsed_seconds(refinement_start);
      timings.refined_candidates += accepted_count;
      timings.total_seconds += elapsed_seconds(total_start);
      feasible.insert(
        feasible.end(), std::make_move_iterator(failed.begin()),
        std::make_move_iterator(failed.end()));
      if (accepted_count == 0) {
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

    planner_timings_t fallback_timings;
    grasp_result_t const pick = problem.pick_grasp_candidates.empty()
      ? fallback_grasps(
          problem.direct.pick, problem.direct, config.direct, fallback_timings)
      : grasp_result_t {
          .status = solve_status_e::success,
          .candidates = problem.pick_grasp_candidates,
          .selected_index = std::size_t {0},
          .failure = {},
        };
    grasp_result_t const place = problem.place_grasp_candidates.empty()
      ? fallback_grasps(
          problem.direct.place, problem.direct, config.direct, fallback_timings)
      : grasp_result_t {
          .status = solve_status_e::success,
          .candidates = problem.place_grasp_candidates,
          .selected_index = std::size_t {0},
          .failure = {},
        };
    if (
      pick.status != solve_status_e::success ||
      place.status != solve_status_e::success) {
      plan_result_t result {
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
        .timings = {},
      };
      result.timings = direct.timings;
      accumulate_timings(result.timings, fallback_timings);
      return result;
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
    planner_timings_t combined_timings = direct.timings;
    accumulate_timings(combined_timings, fallback_timings);
    accumulate_timings(combined_timings, fallback.timings);
    fallback.timings = combined_timings;
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
