#include "bindings_common.hpp"

#include <stacking_core/planner/types.hpp>

#include <nanobind/stl/vector.h>

#include <cstdint>
#include <utility>
#include <vector>

namespace nb = nanobind;

namespace stacking_core::python {

void bind_scene(nb::module_ &module) {
  nb::class_<SceneView>(module, "SceneView")
      .def(nb::new_([](python_scene_snapshot_t const &snapshot,
                       std::vector<std::uint64_t> const &ids) {
             return SceneView{snapshot.value, entity_ids(ids)};
           }),
           nb::arg("snapshot"), nb::arg("entity_ids"))
      .def_prop_ro("snapshot",
                   [](SceneView const &scene) {
                     return python_scene_snapshot_t{.value =
                                                        scene.snapshotPtr()};
                   })
      .def_prop_ro("frame_id",
                   [](SceneView const &scene) { return scene.frame().value(); })
      .def_prop_ro("entity_ids",
                   [](SceneView const &scene) {
                     return python_entity_ids(scene.entityIds());
                   })
      .def_prop_ro("body_count", &SceneView::bodyCount)
      .def("contains", [](SceneView const &scene, std::uint64_t id) {
        return scene.contains(EntityId{id});
      });

  nb::class_<phase_scene_t>(module, "PhaseScene")
      .def(nb::new_([](SceneView scene, std::uint64_t target_id) {
             return phase_scene_t{
                 .scene = std::move(scene),
                 .target = EntityId{target_id},
             };
           }),
           nb::arg("scene"), nb::arg("target_id"))
      .def(nb::new_([](python_scene_snapshot_t const &snapshot,
                       std::vector<std::uint64_t> const &ids,
                       std::uint64_t target_id) {
             return phase_scene_t{
                 .scene = SceneView{snapshot.value, entity_ids(ids)},
                 .target = EntityId{target_id},
             };
           }),
           nb::arg("snapshot"), nb::arg("entity_ids"), nb::arg("target_id"))
      .def_rw("scene", &phase_scene_t::scene)
      .def_prop_rw(
          "target_id",
          [](phase_scene_t const &phase) { return phase.target.value(); },
          [](phase_scene_t &phase, std::uint64_t value) {
            phase.target = EntityId{value};
          });
}

} // namespace stacking_core::python
