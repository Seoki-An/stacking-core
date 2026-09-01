#pragma once

#include <stacking_core/posegen/types.hpp>

#include <memory>

namespace stacking_core {

class PoseGenerator {
public:
  explicit PoseGenerator(posegen_config_t config = {});
  ~PoseGenerator();

  PoseGenerator(PoseGenerator const&) = delete;
  PoseGenerator& operator=(PoseGenerator const&) = delete;
  PoseGenerator(PoseGenerator&&) noexcept;
  PoseGenerator& operator=(PoseGenerator&&) noexcept;

  [[nodiscard]] posegen_config_t const& config() const noexcept;
  void setConfig(posegen_config_t config);

  [[nodiscard]] posegen_result_t solve(posegen_problem_t const& problem);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace stacking_core
