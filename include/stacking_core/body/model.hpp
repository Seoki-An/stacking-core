#pragma once

#include <stacking_core/geometry.hpp>

#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

namespace stacking_core {

struct inertial_t {
  pose_t body_from_inertial;
  Scalar mass = 1.0;
  Matrix3 inertia = Matrix3::Identity();
};

struct body_model_config_t {
  BodyModelId id;
  std::optional<inertial_t> inertial;
  std::vector<geometry_config_t> geometries;
};

class BodyModel {
public:
  explicit BodyModel(body_model_config_t config);

  BodyModel(BodyModel const&) = delete;
  BodyModel& operator=(BodyModel const&) = delete;
  BodyModel(BodyModel&&) = delete;
  BodyModel& operator=(BodyModel&&) = delete;

  [[nodiscard]] BodyModelId id() const noexcept {
    return id_;
  }

  [[nodiscard]] bool hasInertial() const noexcept {
    return inertial_.has_value();
  }

  [[nodiscard]] inertial_t const& inertial() const {
    if (!inertial_.has_value()) {
      throw std::logic_error("body model has no inertial data");
    }
    return *inertial_;
  }

  [[nodiscard]] std::size_t geometryCount() const noexcept {
    return geometries_.size();
  }

  // Immutable body-frame union of all geometry bounds.
  [[nodiscard]] BoundingVolume const& boundingVolume() const noexcept {
    return bounding_volume_;
  }

  [[nodiscard]] Geometry const& geometry(std::size_t index) const;
  [[nodiscard]] Geometry const& geometry(GeometryId id) const;
  [[nodiscard]] Geometry const* findGeometry(GeometryId id) const noexcept;

private:
  BodyModelId id_;
  std::optional<inertial_t> inertial_;
  std::vector<std::unique_ptr<Geometry>> geometries_;
  BoundingVolume bounding_volume_;
};

}  // namespace stacking_core
