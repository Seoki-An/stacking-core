#pragma once

#include <stacking_core/types.hpp>

#include <filesystem>
#include <vector>

namespace stacking_core::detail {

struct dsf_mesh_t {
  Matrix3X nodes;
  int sharpness = 20;
  Scalar friction = 1.0;
};

[[nodiscard]] std::vector<dsf_mesh_t> load_dsf_meshes(
  std::filesystem::path const& path);

}  // namespace stacking_core::detail
