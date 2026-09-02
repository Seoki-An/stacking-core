#include "bindings_common.hpp"

namespace nb = nanobind;

NB_MODULE(_native, module) {
  module.doc() = "Python bindings for stacking-core";

  stacking_core::python::bind_simulation(module);
  stacking_core::python::bind_scene(module);
  stacking_core::python::bind_kinematics(module);
  stacking_core::python::bind_planner_types(module);
  stacking_core::python::bind_planner_grasp(module);
  stacking_core::python::bind_planner_motion(module);
  stacking_core::python::bind_planner_pick_place(module);
}
