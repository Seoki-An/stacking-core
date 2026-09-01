#include <stacking_core/io/urdf.hpp>
#include <stacking_core/kinematics.hpp>

#include <Eigen/Geometry>

#include <filesystem>
#include <memory>
#include <stdexcept>

namespace {

  void require(bool condition) {
    if (!condition) {
      throw std::runtime_error("URDF loader test requirement failed");
    }
  }

  std::filesystem::path asset(std::string const& name) {
    return std::filesystem::path {__FILE__}.parent_path() / "assets" / name;
  }

}  // namespace

int main() {
  using namespace stacking_core;

  UrdfModel model = load_urdf_model(asset("robot.urdf"));
  require(model.name() == "test_robot");
  require(model.kinematics().linkCount() == 6);
  require(model.kinematics().jointCount() == 4);
  require(model.kinematics().degreeOfFreedomCount() == 2);
  require(model.links().size() == 6);

  LinkId const world = model.kinematics().findLink("world")->id;
  LinkId const base = model.kinematics().findLink("base")->id;
  LinkId const arm = model.kinematics().findLink("arm")->id;
  LinkId const finger = model.kinematics().findLink("finger")->id;
  require(model.kinematics().roots().size() == 2);
  require(model.kinematics().findJoint("floating_base") == nullptr);

  BodyModel const& world_body = model.bodyModel(world);
  require(!world_body.hasInertial());
  require(world_body.geometryCount() == 0);

  BodyModel const& base_body = model.bodyModel(base);
  require(base_body.hasInertial());
  require(base_body.inertial().mass == 2.0);
  require(base_body.inertial().body_from_inertial.position.isApprox(
    Vector3 {0.1, 0.2, 0.3}));
  Quaternion const expected_inertial_rotation {
    Eigen::AngleAxis<Scalar> {0.5, Vector3::UnitZ()}};
  require(base_body.inertial().body_from_inertial.orientation.isApprox(
    expected_inertial_rotation));
  require(
    base_body.inertial().inertia.diagonal().isApprox(Vector3 {2.0, 3.0, 4.0}));

  require(base_body.geometryCount() == 1);
  Geometry const& geometry = base_body.geometry(0);
  require(geometry.type() == geometry_type_e::dsf_vert);
  require(geometry.material().friction == 0.7);
  require(
    geometry.bodyFromGeometry().position.isApprox(Vector3 {2.0, 2.0, 3.0}));
  auto const* dsf = dynamic_cast<DsfVertGeometry const*>(&geometry);
  require(dsf != nullptr);
  require(dsf->nodes().cols() == 5);
  require(dsf->sharpness() == 5);
  require(dsf->nodes().rowwise().mean().isZero(1e-12));

  kinematic_joint_t const& shoulder = *model.kinematics().findJoint("shoulder");
  kinematic_joint_t const& mimic =
    *model.kinematics().findJoint("finger_mimic");
  require(shoulder.axis.isApprox(Vector3::UnitZ()));
  require(shoulder.limit->lower == -1.0);
  require(mimic.mimic->source == shoulder.id);
  require(model.kinematics().findJoint("wheel_joint")->limit == std::nullopt);

  UrdfModel preamble_model = load_urdf_model(asset("preamble.urdf"));
  LinkId const target = preamble_model.kinematics().findLink("target")->id;
  auto const& preamble_dsf = static_cast<DsfVertGeometry const&>(
    preamble_model.bodyModel(target).geometry(0));
  require(preamble_dsf.sharpness() == 11);
  require(preamble_dsf.material().friction == 0.35);

  KinematicState state {FrameId {9}, model.kinematicsPtr()};
  state.setFrameFromRoot(world, pose_t {});
  state.setFrameFromRoot(
    base, pose_t {Vector3 {10.0, 0.0, 0.0}, Quaternion::Identity()});
  state.setPosition(shoulder.id, 0.5);
  KinematicSnapshot const snapshot = forward_kinematics(state);
  require(
    snapshot.frameFromLink(arm).position.isApprox(Vector3 {11.0, 0.0, 0.0}));
  require(state.position(mimic.id) == -0.4);
  Quaternion const expected_finger_rotation {
    Eigen::AngleAxis<Scalar> {0.1, Vector3::UnitZ()}};
  require(snapshot.frameFromLink(finger).orientation.isApprox(
    expected_finger_rotation));

  bool rejected_invalid = false;
  try {
    UrdfModel invalid = load_urdf_model(asset("invalid.urdf"));
    (void)invalid;
  } catch (UrdfError const&) {
    rejected_invalid = true;
  }
  require(rejected_invalid);
}
