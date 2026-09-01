# AGENTS.md — stacking-core

## Project Direction

- `stacking-core` is a clean-break refactor of `diffsim`; preserving the old
  API is not a goal.
- `stacking-planner` and `stacking-tabletop` will be refactored later. Use their
  current workflows to identify requirements and avoid structural conflicts,
  not as interfaces that must remain compatible.
- Do not migrate the deprecated `poseinit` module. Reuse an individual utility
  from it only when that utility is independently useful.

## Core Architecture

- Keep foundational types in core: stable entity IDs, transforms, body models
  and instances, inertial data, and the `Geometry` hierarchy (`DsfVertGeometry`,
  `PlaneGeometry`, and `PointGeometry`).
- Separate immutable body/geometry models from per-scene pose, motion, and
  mobility state.
- Define one canonical, frame-explicit `SceneSnapshot` in core. Give each
  solver its own typed problem, configuration, scene view, and result instead
  of duplicating scene implementations or relying on string-selected modes.
- Treat target, boundary, fixed, and optimizable status as roles for a specific
  solve unless they are intrinsic physical state.
- Solvers consume scene data and return explicit results or state updates; they
  must not become a second hidden source of scene truth.
- Keep `simulation` and `posegen` as sibling modules outside `src/core`. They
  depend on the `stacking-core::core` target and expose independent
  `stacking-core::simulation` and `stacking-core::posegen` targets.
- Keep hard-coded robot algorithms in optional extension targets. Generic
  planner targets may be dependencies of such extensions, but must not depend
  on or include them.

## Code Style

- Be as simple as possible.
- Prefer easily interpretable code.
- Do not add options the user did not request.
- Follow the `diffsim` naming convention: classes use `PascalCase`; structs and
  plain data types use `snake_case_t`; enums use `snake_case_e`; free and
  internal functions, fields, and local variables use `snake_case`.
- Prefer concise names for unambiguous algorithm-local quantities: `dir`,
  `ref`, `iter`, `max`, `grad`, `tol`, and `proj`. Mathematical matrix notation
  such as `V_t` is appropriate when it matches the algorithm. Keep public and
  domain-level names descriptive.

## Algorithm Style

- Always remember Occam's razor when changing or adding planning algorithms.
- Do not reinvent the wheel: if suitable tools are already implemented in the
  source, use them.
