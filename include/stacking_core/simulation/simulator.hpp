#pragma once

#include <stacking_core/simulation/types.hpp>

#include <cstddef>
#include <memory>

namespace stacking_core {

class Simulator {
public:
  explicit Simulator(simulation_config_t config = {});
  ~Simulator();

  Simulator(Simulator const&) = delete;
  Simulator& operator=(Simulator const&) = delete;
  Simulator(Simulator&&) noexcept;
  Simulator& operator=(Simulator&&) noexcept;

  [[nodiscard]] simulation_config_t const& config() const noexcept;

  // Consumes one canonical scene snapshot and returns the next one. Model
  // objects remain shared and immutable; pose and motion state are copied.
  [[nodiscard]] simulation_result_t step(
    SceneSnapshot const& scene, Scalar dt);

  // Exactly repeats step() while threading each returned snapshot into the
  // next call. A zero count returns an owned copy of the input snapshot and
  // leaves the solver warm start unchanged. For a non-zero count, contacts
  // and solver statistics describe the final step.
  [[nodiscard]] simulation_result_t step_n(
    SceneSnapshot const& scene, Scalar dt, std::size_t count);

  void clearWarmStart();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace stacking_core
