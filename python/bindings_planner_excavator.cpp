#include "bindings_common.hpp"

#include <stacking_core/planner/excavator.hpp>
#include <stacking_core/planner/motion.hpp>

#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace stacking_core::python {

void bind_planner_excavator(nb::module_ &module) {
  module.def(
      "configure_excavator_ik",
      [](motion_robot_t &robot, std::vector<std::string> const &joint_names,
         std::string const &end_link, array_t const &end_from_tool) {
        auto const &model = robot.initial_state.modelPtr();
        if (joint_names.size() != 6) {
          throw std::invalid_argument("excavator IK requires six joint names");
        }
        std::array<JointId, 6> joints;
        for (std::size_t i = 0; i < joints.size(); ++i) {
          auto const *joint = model->findJoint(joint_names[i]);
          if (joint == nullptr) {
            throw std::invalid_argument("unknown excavator joint: " + joint_names[i]);
          }
          joints[i] = joint->id;
        }
        auto const *end = model->findLink(end_link);
        if (end == nullptr) {
          throw std::invalid_argument("unknown excavator end link: " + end_link);
        }
        auto initializer = std::make_shared<ExcavatorIkInitializer>(
            model, excavator_ik_chain_t{
                       .swing = joints[0], .boom = joints[1], .arm = joints[2],
                       .bucket = joints[3], .tilt = joints[4], .rotate = joints[5],
                       .end_link = end->id,
                       .end_from_task = pose_from_array(end_from_tool, "end_from_tool"),
                   });
        // Own the native initializer in the callback: no Python callback or GIL
        // is involved when parallel native grasp/motion workers request seeds.
        LinkId const tool_link = robot.tool_link;
        robot.ik_initializer =
            [initializer = std::move(initializer), tool_link](
                KinematicState const &state, LinkId link,
                pose_t const &frame_from_link) -> std::optional<Eigen::VectorXd> {
          if (link != tool_link) {
            return std::nullopt;
          }
          auto result = initializer->seed(state, frame_from_link);
          if (result.status != solve_status_e::success) {
            return std::nullopt;
          }
          return result.positions;
        };
      },
      nb::arg("robot"), nb::arg("joint_names"), nb::arg("end_link"),
      nb::arg("end_from_tool"));

  module.def(
      "excavator_ik_seed",
      [](motion_robot_t const &robot,
         array_t const &frame_from_tool) -> std::optional<Eigen::VectorXd> {
        if (!robot.ik_initializer) {
          throw std::invalid_argument("robot has no IK initializer");
        }
        return robot.ik_initializer(
            robot.initial_state, robot.tool_link,
            pose_from_array(frame_from_tool, "frame_from_tool"));
      },
      nb::arg("robot"), nb::arg("frame_from_tool"));
}

} // namespace stacking_core::python
