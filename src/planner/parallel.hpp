#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <exception>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace stacking_core::detail {

  inline std::size_t planner_worker_count(
    int configured, std::size_t task_count) {
    if (task_count == 0) {
      return 0;
    }
    unsigned int const hardware = std::thread::hardware_concurrency();
    std::size_t const requested = configured == 0
      ? std::max(1U, hardware)
      : static_cast<std::size_t>(configured);
    return std::min(requested, task_count);
  }

  template <typename Stop, typename Function>
  void planner_parallel_for(
    std::size_t task_count, int configured_workers, Stop&& stop,
    Function&& function) {
    std::size_t const worker_count =
      planner_worker_count(configured_workers, task_count);
    if (worker_count == 0) {
      return;
    }
    if (worker_count == 1) {
      for (std::size_t i = 0; i < task_count && !stop(); ++i) {
        function(i);
      }
      return;
    }

    std::atomic<std::size_t> next {0};
    std::atomic<bool> failed {false};
    std::exception_ptr error;
    std::mutex error_mutex;
    auto work = [&]() {
      while (!failed.load(std::memory_order_relaxed) && !stop()) {
        std::size_t const i = next.fetch_add(1, std::memory_order_relaxed);
        if (i >= task_count) {
          break;
        }
        try {
          function(i);
        } catch (...) {
          {
            std::lock_guard lock {error_mutex};
            if (error == nullptr) {
              error = std::current_exception();
            }
          }
          failed.store(true, std::memory_order_relaxed);
        }
      }
    };

    std::vector<std::thread> workers;
    workers.reserve(worker_count);
    try {
      for (std::size_t i = 0; i < worker_count; ++i) {
        workers.emplace_back(work);
      }
    } catch (...) {
      failed.store(true, std::memory_order_relaxed);
      for (std::thread& worker : workers) {
        worker.join();
      }
      throw;
    }
    for (std::thread& worker : workers) {
      worker.join();
    }
    if (error != nullptr) {
      std::rethrow_exception(error);
    }
  }

  template <typename Function>
  void planner_parallel_for(
    std::size_t task_count, int configured_workers, Function&& function) {
    planner_parallel_for(
      task_count, configured_workers, [] { return false; },
      std::forward<Function>(function));
  }

}  // namespace stacking_core::detail
