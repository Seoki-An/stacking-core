#pragma once

#include <stacking_core/body/model.hpp>

#include <cstddef>
#include <vector>

namespace stacking_core {

struct envelope_support_t: support_t {
  GeometryId source_geometry;
  bool has_tie = false;
};

struct diffable_envelope_support_t: diffable_support_t {
  GeometryId source_geometry;
  bool has_tie = false;
};

// Non-owning support view of the convex hull of all DSF-Vert geometries in a
// body model. The body model must outlive the envelope.
class ConvexSupportEnvelope {
public:
  explicit ConvexSupportEnvelope(BodyModel const& body);

  [[nodiscard]] BodyModelId bodyModelId() const noexcept {
    return body_model_id_;
  }

  [[nodiscard]] std::size_t geometryCount() const noexcept {
    return geometries_.size();
  }

  // Returns max_i h_i(dir_frame), with the point expressed in frame.
  // At a tie, source_geometry and its Jacobian come from the first tied
  // geometry in BodyModel order, but the envelope itself is nondifferentiable.
  [[nodiscard]] envelope_support_t support(
    Vector3 const& dir_frame, pose_t const& frame_from_body) const;

  [[nodiscard]] diffable_envelope_support_t diffable_support(
    Vector3 const& dir_frame, pose_t const& frame_from_body) const;

private:
  BodyModelId body_model_id_;
  std::vector<DsfVertGeometry const*> geometries_;
};

}  // namespace stacking_core
