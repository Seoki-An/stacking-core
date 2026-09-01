#pragma once

#include <stacking_core/transform.hpp>
#include <stacking_core/types.hpp>
#include <stacking_core/geometry/bounding_volume.hpp>

namespace stacking_core {

enum class geometry_type_e {
  point,
  plane,
  dsf_vert,
};

struct material_t {
  Scalar friction = 1.0;
};

struct geometry_properties_t {
  GeometryId id;
  pose_t body_from_geometry;
  material_t material;
};

class Geometry {
public:
  virtual ~Geometry() = default;

  Geometry(Geometry const&) = delete;
  Geometry& operator=(Geometry const&) = delete;
  Geometry(Geometry&&) = delete;
  Geometry& operator=(Geometry&&) = delete;

  [[nodiscard]] GeometryId id() const noexcept {
    return id_;
  }

  [[nodiscard]] pose_t const& bodyFromGeometry() const noexcept {
    return body_from_geometry_;
  }

  [[nodiscard]] material_t const& material() const noexcept {
    return material_;
  }

  // Immutable axis-aligned bound in the geometry's local frame.
  [[nodiscard]] BoundingVolume const& boundingVolume() const noexcept {
    return bounding_volume_;
  }

  [[nodiscard]] virtual geometry_type_e type() const noexcept = 0;

protected:
  Geometry(geometry_properties_t properties, BoundingVolume bounding_volume);

private:
  GeometryId id_;
  pose_t body_from_geometry_;
  material_t material_;
  BoundingVolume bounding_volume_;
};

}  // namespace stacking_core
