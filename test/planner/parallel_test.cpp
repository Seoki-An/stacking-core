#include "parallel.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <source_location>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

  void require(
    bool condition,
    std::source_location location = std::source_location::current()) {
    if (!condition) {
      throw std::runtime_error(
        "planner parallel requirement failed at line " +
        std::to_string(location.line()));
    }
  }

  void update_max(std::atomic<int>& max, int value) {
    int previous = max.load(std::memory_order_relaxed);
    while (
      previous < value &&
      !max.compare_exchange_weak(previous, value, std::memory_order_relaxed)) {
    }
  }

}  // namespace

int main() {
  using namespace stacking_core;

  std::atomic<int> active {0};
  std::atomic<int> max_active {0};
  std::atomic<int> completed {0};
  detail::planner_parallel_for(8, 4, [&](std::size_t) {
    int const count = active.fetch_add(1, std::memory_order_relaxed) + 1;
    update_max(max_active, count);
    std::this_thread::sleep_for(std::chrono::milliseconds {5});
    active.fetch_sub(1, std::memory_order_relaxed);
    completed.fetch_add(1, std::memory_order_relaxed);
  });
  require(completed.load() == 8);
  require(max_active.load() >= 2);
  require(max_active.load() <= 4);

  std::atomic<int> accepted {0};
  std::atomic<int> attempted {0};
  detail::planner_parallel_for(
    100, 4, [&] { return accepted.load(std::memory_order_relaxed) >= 1; },
    [&](std::size_t) {
      attempted.fetch_add(1, std::memory_order_relaxed);
      accepted.fetch_add(1, std::memory_order_relaxed);
    });
  require(attempted.load() >= 1);
  require(attempted.load() <= 4);
}
