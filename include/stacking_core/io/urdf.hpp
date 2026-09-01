#pragma once

#include <stacking_core/body/model.hpp>
#include <stacking_core/kinematics/model.hpp>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace stacking_core {

class UrdfError: public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

struct urdf_link_model_t {
  LinkId link;
  std::shared_ptr<BodyModel const> body_model;
};

// Source bundle produced by URDF loading. The kinematic topology and physical
// link models remain immutable and can be shared by many scene instances.
class UrdfModel {
public:
  UrdfModel(
    std::string name,
    std::shared_ptr<KinematicModel const> kinematics,
    std::vector<urdf_link_model_t> links);

  UrdfModel(UrdfModel const&) = delete;
  UrdfModel& operator=(UrdfModel const&) = delete;
  UrdfModel(UrdfModel&&) noexcept = default;
  UrdfModel& operator=(UrdfModel&&) noexcept = default;

  [[nodiscard]] std::string const& name() const noexcept {
    return name_;
  }

  [[nodiscard]] KinematicModel const& kinematics() const noexcept {
    return *kinematics_;
  }

  [[nodiscard]] std::shared_ptr<KinematicModel const> const&
  kinematicsPtr() const noexcept {
    return kinematics_;
  }

  [[nodiscard]] std::span<urdf_link_model_t const> links() const noexcept {
    return links_;
  }

  [[nodiscard]] BodyModel const& bodyModel(LinkId link) const;
  [[nodiscard]] std::shared_ptr<BodyModel const> const& bodyModelPtr(
    LinkId link) const;

private:
  std::string name_;
  std::shared_ptr<KinematicModel const> kinematics_;
  std::vector<urdf_link_model_t> links_;
};

// Loads URDF topology, inertial data, and <collision><geometry><dsf_vert>
// models. Visual and ordinary URDF collision shapes are presentation assets
// and are not converted into stacking-core contact geometry.
[[nodiscard]] UrdfModel load_urdf_model(std::filesystem::path const& path);

}  // namespace stacking_core
