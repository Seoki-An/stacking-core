#include <stacking_core/body/convex_support_envelope.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace stacking_core {
namespace {

template <typename envelope_result_t, typename evaluate_t>
envelope_result_t evaluate_envelope(
  std::vector<DsfVertGeometry const*> const& geometries,
  evaluate_t&& evaluate) {
  envelope_result_t result;
  Scalar second_height = -std::numeric_limits<Scalar>::infinity();
  bool has_result = false;

  for (DsfVertGeometry const* geometry : geometries) {
    auto const candidate = evaluate(*geometry);
    if (!has_result || candidate.h > result.h) {
      if (has_result) {
        second_height = result.h;
      }
      result.h = candidate.h;
      result.s = candidate.s;
      if constexpr (
        std::is_same_v<envelope_result_t, diffable_envelope_support_t>) {
        result.ds_dx = candidate.ds_dx;
        result.ds_dq = candidate.ds_dq;
      }
      result.source_geometry = geometry->id();
      has_result = true;
    } else {
      second_height = std::max(second_height, candidate.h);
    }
  }

  if (geometries.size() > 1) {
    Scalar const scale = std::max(
      {Scalar {1.0}, std::abs(result.h), std::abs(second_height)});
    Scalar const tie_tolerance =
      Scalar {64.0} * std::numeric_limits<Scalar>::epsilon() * scale;
    result.has_tie = result.h - second_height <= tie_tolerance;
  }
  return result;
}

}  // namespace

ConvexSupportEnvelope::ConvexSupportEnvelope(BodyModel const& body)
    : body_model_id_(body.id()) {
  geometries_.reserve(body.geometryCount());
  for (std::size_t index = 0; index < body.geometryCount(); ++index) {
    Geometry const& geometry = body.geometry(index);
    if (geometry.type() == geometry_type_e::dsf_vert) {
      geometries_.push_back(
        static_cast<DsfVertGeometry const*>(&geometry));
    }
  }

  if (geometries_.empty()) {
    throw std::invalid_argument(
      "convex support envelope requires at least one DSF-Vert geometry");
  }
}

envelope_support_t ConvexSupportEnvelope::support(
  Vector3 const& dir_frame, pose_t const& frame_from_body) const {
  return evaluate_envelope<envelope_support_t>(
    geometries_,
    [&](DsfVertGeometry const& geometry) {
      return geometry.support(dir_frame, frame_from_body);
    });
}

diffable_envelope_support_t ConvexSupportEnvelope::diffable_support(
  Vector3 const& dir_frame, pose_t const& frame_from_body) const {
  return evaluate_envelope<diffable_envelope_support_t>(
    geometries_,
    [&](DsfVertGeometry const& geometry) {
      return geometry.diffable_support(dir_frame, frame_from_body);
    });
}

}  // namespace stacking_core
