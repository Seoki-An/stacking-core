# stacking-core

Refactored core library for the stacking project.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

## Use from CMake

After installation, consumers link only the module they need:

```cmake
find_package(stacking-core CONFIG REQUIRED)
target_link_libraries(my-core-target PRIVATE stacking-core::core)
target_link_libraries(my-simulator PRIVATE stacking-core::simulation)
target_link_libraries(my-pose-generator PRIVATE stacking-core::posegen)
target_link_libraries(my-planner PRIVATE stacking-core::planner)
target_link_libraries(
  my-excavator-planner PRIVATE stacking-core::planner-excavator)
```

`stacking-core::simulation`, `stacking-core::posegen`, and
`stacking-core::planner` each depend on `stacking-core::core`. The convenience
target `stacking-core::stacking-core` links the general-purpose modules.

`stacking-core::planner-excavator` is an optional model-specific extension. It
is deliberately not linked by `stacking-core::planner` or the convenience
target.

## Python bindings

The optional Python package exposes immutable body models, body instances,
canonical scene snapshots, simulation configuration, contacts, and solver
statistics:

```bash
python -m pip install -e python
```

The editable build invokes the same CMake targets as the C++ build and uses
nanobind for the native module. To build it directly with CMake, install the
Python build requirements first:

```bash
python -m pip install nanobind
cmake -S . -B build-python \
  -DSTACKING_CORE_BUILD_PYTHON=ON \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-python --target stacking-core-python
PYTHONPATH=build-python/python python -c "import stacking_core"
```

NumPy poses use `[x, y, z, qx, qy, qz, qw]`. DSF nodes use the conventional
`N x 3` array layout. `Simulator.step()` and `step_n()` return a new
`SceneSnapshot`; they never mutate the Python-visible input scene.

## URDF models

URDF loading is part of the library, so consumers do not need a separate URDF
parser dependency:

```cpp
#include <stacking_core/io/urdf.hpp>

auto robot = stacking_core::load_urdf_model("robot.urdf");
auto const& topology = robot.kinematics();
auto const& base_body = robot.bodyModel(topology.findLink("base")->id);
```

The loader imports link inertials, fixed/revolute/prismatic/continuous joints,
mimic relationships, joint limits, and `dsf_vert` collision geometry. A root
floating joint becomes a frame-explicit root pose in `KinematicState`.

To instantiate the model into a scene, provide an explicit entity ID for every
link and call `make_urdf_scene_snapshot()`. The resulting bodies share the
loaded body models and receive their poses from forward kinematics.

## Planner

The planner is a sibling module rather than part of foundational core. Planner
algorithms consume typed problems and return structured status, solver
statistics, and diagnostics.

Inverse kinematics operates directly on a `KinematicState`, so the loaded model,
root poses, joint ordering, and target frame remain explicit:

```cpp
auto const* tool = robot.kinematics().findLink("tool");
stacking_core::KinematicState state {frame_id, robot.kinematicsPtr()};

auto ik = stacking_core::solve_inverse_kinematics(
  stacking_core::inverse_kinematics_problem_t {
    .initial_state = state,
    .link = tool->id,
    .frame_from_link = target_pose,
    .position_only = false,
  });
```

The generic solver preserves diffsim's iterative Levenberg–Marquardt behavior:
swing initialization, the hybrid translational/body-angular residual, adaptive
damping, joint-margin penalty, and stagnation exit. Position-only IK drops the
orientation residual. Excavator-specific closed-form initialization is kept
separate from this robot-independent solver.

The optional excavator extension provides the hard-coded VDK23_CX closed-form
initializer:

```cpp
#include <stacking_core/planner/excavator.hpp>

stacking_core::ExcavatorIkInitializer initializer {
  robot.kinematicsPtr(),
  excavator_chain,
};
auto seed = initializer.seed(state, target_pose);
```

The chain identifies all six joints, the end link, and the explicit
`end_from_task` transform. Construction verifies the serial topology, joint
axes, and VDK23_CX origin signature before the hard-coded DH equations can run.
A successful seed is refined by `solve_inverse_kinematics()` with
`inverse_kinematics_initialization_e::provided`. The generic planner contains
no excavator dimensions, frame corrections, or dependency on this extension.

Stable-pose generation operates on an immutable `BodyModel`. It builds one
smooth DSF envelope over the body's DSF geometries, then solves support-height
equilibria on the sphere with the same Hessian-based Riemannian trust-region
method as diffsim:

```cpp
auto stable = stacking_core::solve_stable_poses(
  stacking_core::stable_pose_problem_t {
    .body_model = robot.bodyModelPtr(target_link),
    .com_offset_body = stacking_core::Vector3::Zero(),
  },
  stacking_core::stable_pose_config_t {.sampling_level = 3});
```

Each result contains the body-frame resting direction, basin cone angle, and
body-frame support point. OBJ metadata such as `# sharpness: 60` and `# mu:` is
honored both before and inside object declarations, preserving legacy DSF
parameters during URDF loading.

Free and grasped motion planning share one robot description and optimizer,
but use distinct typed problems. The obstacle scene remains immutable; moving
link collision bodies are bound explicitly by link and entity ID:

```cpp
stacking_core::motion_robot_t motion_robot {
  .initial_state = state,
  .tool_link = tool_link,
  .link_from_tool = link_from_tool,
  .gripper_opening = opening,
  .ik_initializer = {},
  .collision_bodies = collision_links,
  .self_collision_pairs = self_collision_pairs,
};

auto free = stacking_core::solve_free_motion(
  stacking_core::free_motion_problem_t {
    .scene = obstacle_scene,
    .robot = motion_robot,
    .waypoints = {{
      .goal = stacking_core::joint_goal_t {.positions = q_goal},
      .steps = 20,
    }},
  });
```

`solve_grasped_motion()` additionally receives an `attachment_t` referencing a
body already present in the scene. It derives every carried-body pose from the
kinematic link instead of introducing a second mutable scene. Joint and
link-pose goals use the same waypoint type; link-pose goals are resolved by the
generic IK solver. A robot-specific seed can be supplied through
`ik_initializer` without coupling the planner to the optional excavator
extension.

Regrasp planning composes stable-pose generation, IK, and the motion solvers.
Grasp generation stays a separate stage: callers provide scored pick-side and
place-side `grasp_candidate_t` collections, and the solver preserves each
candidate's body-relative grasp at the intermediate handoff pose:

```cpp
auto plan = stacking_core::solve_regrasp(
  stacking_core::regrasp_problem_t {
    .pick = pick_phase,
    .handoff = handoff_phase,
    .place = place_phase,
    .robot = motion_robot,
    .pick_grasps = pick_grasps.candidates,
    .place_grasps = place_grasps.candidates,
    .handoff_position = {x, y, table_height},
  });
```

Each successful candidate contains eight ordered segments: pick approach and
retreat, handoff placement and retreat, handoff approach and pickup, then place
approach and retreat. Acquire/release events identify the exact segment sample.
The two sides of the handoff are solved independently before score pairing, so
`A x B` grasp combinations require `A + B` motion-leg evaluations. Stable-face
yaw sampling and target-boundary checks remain explicit in `regrasp_config_t`.

`phase_scene_t` assigns a target role to a `SceneView`. Direct planning receives
explicit pick and place phases, while regrasp planning additionally receives a
handoff phase. Free and carried motion use separate problem types but share the
same trajectory representation. A `plan_result_t` stores self-contained
candidates and a selected index instead of duplicating the selected plan into
parallel result arrays.

## Simulation state evolution

`Simulator` consumes an immutable `SceneSnapshot` and returns the next owned
snapshot. It does not keep a second mutable scene internally:

```cpp
stacking_core::Simulator simulator;
auto next = simulator.step(scene, 0.01);
auto final = simulator.step_n(*next.snapshot, 0.01, 100);
```

`step_n` is exactly repeated `step`: contact detection, warm starting, solving,
and pose integration still occur at every timestep. A zero count returns an
owned copy of the input snapshot; a non-zero result reports contacts and solver
statistics from its final step. Reusing a retained snapshot is the reset
operation; call `clearWarmStart()` as well when the new state is unrelated to
the preceding contact sequence.

Mobility has explicit evolution semantics: static bodies retain pose, motion,
and mobility unchanged; kinematic bodies integrate their prescribed motion and
contribute that full known motion to relative contact velocity; dynamic bodies
receive solver-updated motion before pose integration. Linear velocity is in
the scene frame and angular velocity is in the body frame.

## DSF pose generation

`PoseGenerator` optimizes one candidate pose using the DSF force-closure
objective. Candidate, target, and boundary status are roles in a typed problem;
all referenced bodies remain in one canonical `SceneSnapshot`:

```cpp
stacking_core::PoseGenerator posegen;
auto result = posegen.solve(stacking_core::posegen_problem_t {
  .scene = scene_view,
  .candidate = candidate_id,
  .targets = target_ids,
  .boundaries = boundary_ids,
});

if (!result.solver.converged) {
  // Inspect result.solver.grad_norm and result.solver.force_solver.
}
```

The solver includes ground and multi-body contact forces, boundary reactions,
force equilibrium, target containment, and the Riemannian trust-region pose
update. The contact-force subproblem is solved by a consensus ADMM over the
force graph. A primal-dual interior-point backend solves the same subproblem
directly over one force per contact, for comparing the two:

```cpp
posegen_config_t config;
config.force_solver.method = stacking_core::posegen_force_solver_e::
  interior_point;
```

The graph solver remains the default. The interior-point backend reuses the
same iteration limit and tolerances, reports Newton steps in
`force_solver.iters`, and writes node forces back in the graph layout, so
results, contact forces, and the pose gradient are unchanged apart from solver
accuracy.

Two contacts couple only when they share a body, so the condensed Newton matrix
inherits that sparsity: a contact chain gives a block-tridiagonal system. The
pattern is fixed for a given contact set, so it is analyzed once per solve and
only the per-contact diagonal blocks are rewritten between Newton steps. Consecutive solves reuse immutable scene contacts and force-graph warm
state when only the candidate pose changes; scene or contact-defining
configuration changes invalidate that cache. Warm starting is what makes the
graph solver fast here: a pose optimization calls the force solve tens of
times on a slightly perturbed problem, and the graph solver amortizes across
those calls in a way an interior-point method cannot. The result reports outer and
force-solver convergence, residuals, objective evaluations, and scene-graph
reuse counts.

The implementation is numerically checked against diffsim for ground-only,
ordinary stacking, target-containment, and multi-contact cases. Boundary
behavior uses the intended contact-frame normal column, correcting diffsim's
invalid `col(-1)` cone-gradient access, and the cone KKT force gradient is
divided by the smoothed tangential magnitude rather than by one built from a
tangent and the normal. Both corrections only move contacts that carry
tangential load. Default objective values follow the `stacking-planner` and
`stacking-tabletop` workflows where those agree (`eps_target`, `k_potential`);
parameters those projects set per robot scale, such as `eps_gap`, `eps_comp`,
`k_box`, and `w_box`, keep their scale-neutral values and remain the consumer's
choice.

The force-solver tolerance is tighter than diffsim's, at `tol_abs = 1e-4`.
Contact forces feed the pose gradient directly, and at diffsim's `1e-3` they
carry percent-level error, which the trust region then spends iterations
chasing. The trust-region tolerance stays at diffsim's `1e-8` rather than the
`1e-12` both consumers use: `1e-12` exits only when the trust region collapses,
which an accurate gradient does not cause, so the two settings have to be
chosen together. Measured over contact chains of 1-64 bodies and rows of 2-6
contacts, the pair is faster than either the legacy or the all-tight
combination and leaves no scene at its outer iteration limit.

The point-cloud posegen variant and the deprecated `poseinit` module are not
part of stacking-core.
