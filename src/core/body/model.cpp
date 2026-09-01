#include <stacking_core/body/model.hpp>

#include <Eigen/Cholesky>

#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace stacking_core {
namespace {

void validate(inertial_t const& inertial) {
  if (!is_valid(inertial.body_from_inertial)) {
    throw std::invalid_argument("body inertial pose must be valid");
  }
  if (!std::isfinite(inertial.mass) || inertial.mass <= 0.0) {
    throw std::invalid_argument("body mass must be finite and positive");
  }
  if (!inertial.inertia.allFinite()) {
    throw std::invalid_argument("body inertia must be finite");
  }
  if (!inertial.inertia.isApprox(inertial.inertia.transpose(), 1e-12)) {
    throw std::invalid_argument("body inertia must be symmetric");
  }
  Eigen::LLT<Matrix3> const decomposition {inertial.inertia};
  if (decomposition.info() != Eigen::Success) {
    throw std::invalid_argument("body inertia must be positive definite");
  }
}

}  // namespace

BodyModel::BodyModel(body_model_config_t config)
    : id_(config.id), inertial_(std::move(config.inertial)) {
  if (!id_.valid()) {
    throw std::invalid_argument("body model ID must be valid");
  }
  if (inertial_.has_value()) {
    validate(*inertial_);
  }

  geometries_.reserve(config.geometries.size());
  std::unordered_set<GeometryId> geometry_ids;
  for (geometry_config_t& geometry_config : config.geometries) {
    std::unique_ptr<Geometry> geometry =
      make_geometry(std::move(geometry_config));
    if (!geometry_ids.emplace(geometry->id()).second) {
      throw std::invalid_argument("body model geometry IDs must be unique");
    }
    bounding_volume_ = merge(
      bounding_volume_,
      transformed(
        geometry->boundingVolume(), geometry->bodyFromGeometry()));
    geometries_.push_back(std::move(geometry));
  }
}

Geometry const& BodyModel::geometry(std::size_t index) const {
  return *geometries_.at(index);
}

Geometry const& BodyModel::geometry(GeometryId id) const {
  Geometry const* result = findGeometry(id);
  if (result == nullptr) {
    throw std::out_of_range("body model does not contain the geometry ID");
  }
  return *result;
}

Geometry const* BodyModel::findGeometry(GeometryId id) const noexcept {
  for (std::unique_ptr<Geometry> const& geometry : geometries_) {
    if (geometry->id() == id) {
      return geometry.get();
    }
  }
  return nullptr;
}

}  // namespace stacking_core
