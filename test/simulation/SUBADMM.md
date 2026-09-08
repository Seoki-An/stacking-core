# Simulation subADMM residual / penalty audit

Scope: simulation `ConstraintSolver`, compared with stacking-core commit
`9ab54e8` (not a new comparison against legacy diffsim). Posegen, collision,
contact projectors, row scaling, solver defaults, and per-body 6x6 solves are
unchanged.

## 1. Residual and iteration consistency

Let `Jbar = S J` and `ebar = S e` denote the existing row-scaled contact
coordinates. The solver stores the scaled contact multiplier `lambda`
and per-body auxiliary `y = beta Jbar v - lambda`.

The old node diagnostic was

```text
rhs - p - sum Jbar' lambda
  = M v - p - sum Jbar' lambda + beta sum Jbar' Jbar v.
```

The last term belongs to the augmented solve, not momentum stationarity. It
can remain nonzero at a valid moving-contact solution, or cancel the true
residual at an incorrect velocity. A scalar separating contact with
`M=J=beta=1, p=1` previously returned `v=0.5` with zero reported dual
residual, although the analytical solution is `v=1, lambda=0`.

Both residuals are now evaluated after a complete sweep:

```text
primal = max_contact ||S^-1 (lambda_old - lambda_new) / beta||_inf
dual   = max_body    ||M v_new - p - sum Jbar' lambda_new||_inf
```

The dual relative scale uses generalized momentum:
`max(||Mv||_inf, ||p||_inf, ||sum Jbar' lambda||_inf)`, not the
row-scaled multiplier norm. Primal scaling uses unscaled `e` and `Jv`.
The reported diagnostics therefore describe the returned impulse/velocity
pair, including at the iteration cap.

The minimum of two sweeps is retained. Cold auxiliaries are zero, so the
first projected impulse can be unchanged before the projection has seen
contact velocity; zero impulse change alone must not certify that first
sweep. The primal diagnostic remains an iterate-change criterion, not an
independent contact-feasibility certificate.

## 2. Beta transition ordering

Each sweep uses one beta for projection, RHS assembly, factorization, and
auxiliary update. Convergence/stagnation is checked after that sweep.
On an update boundary, the residual-ratio heuristic proposes a clamped beta.
Before the next sweep:

```text
y <- (beta_new / beta_old) (y + lambda) - lambda
beta <- beta_new
factorize M + beta sum Jbar' Jbar
```

This preserves the represented `Jbar v` and multiplier. The old code changed
the matrix after assembling an old-beta RHS, then deferred an auxiliary
conversion until after another projection. Cached auxiliaries are now also
converted if the warm-start beta is clamped by new configuration bounds.
No unused beta update is made after the final allowed sweep.

Unit tests cover separating motion, returned residuals at the cap, analytical
Coulomb sliding, the cold-start guard, a moving two-mass contact, warm-beta
clamping, and a hand-calculated adaptive transition.

## 3. Frozen-problem comparison

Build/run from the project root (use a Release build for timings):

```sh
cmake --build build-modular --target stacking-core-constraint-solver-benchmark -j 2
build-modular/test/simulation/stacking-core-constraint-solver-benchmark 1e-8
build-modular/test/simulation/stacking-core-constraint-solver-benchmark 1e-4
```

The optional target emits CSV and is excluded from the default build/CTest.
There are 240 cases per tolerance: separating/sliding single-body contacts,
1/2/4/8/16-body vertical chains, and chains with alternating masses 0.1 and
10; fixed/adaptive beta; initial beta 0.01/0.1/1/10/100; cold/warm starts.
Jacobians are dt-scaled with dt=0.005 and zero lever arms; dynamics_scale=1,
constraint_scale_reference=0.001. Stagnation stopping is disabled for this comparison, with the
same 2000-sweep cap. Warm means a second solve of the *identical* frozen
problem, retaining the first solve's beta and auxiliaries. Timings are medians
of five repetitions, excluding fixture construction.

The harness independently reconstructs momentum error, checks the production
contact projection's fixed-point error, and compares velocity to an analytical
solution. The radial Coulomb projector holds the positive normal impulse
fixed and clamps tangents; it is **not** the Euclidean SOCP projector. For the
sliding fixture, the analytical velocity is (0.07057, 0, 0).

Selected cold-start iteration counts at tol_abs=1e-8, tol_rel=1e-9, beta_init=1:

| Frozen problem | Old adaptive | Corrected adaptive | Corrected fixed beta=1 |
|---|---:|---:|---:|
| Separating body | 2 (false convergence) | 3 | 3 |
| Sliding body | 2000 (cap) | 107 | 2000 (cap) |
| 1-body chain | 36 | 95 | 2000 (cap) |
| 8-body chain | 360 | 313 | 2000 (cap) |
| 16-body chain | 584 | 566 | 2000 (cap) |
| 16-body, 100:1 mass ratio | 2000 (cap) | 2000 (cap) | 2000 (cap) |

For the separating fixture, true momentum error fell from 9.99e-4 to
9.97e-10. The corrected 16-body equal-mass chain has momentum error 9.70e-9
and max velocity error 2.68e-8. The heterogeneous 16-body chain still has
momentum error 4.01e-7 and max velocity error 8.82e-6 at the cap.

These are correctness/conditioning results, not a universal speedup:
some small problems take more iterations. Fixed beta is strongly
problem-dependent; adaptive beta helps the longer chains. Larger initial
beta helps some cold cases, but not consistently across all cases and warm
starts, so no default beta was changed.

## State-evolution checks

All existing release-transient stability ceilings are unchanged. Additional
before/after diagnostics reused the DSF cube setup in `stability_test.cpp`
(half extent 0.25, sharpness 60, mass 1, inertia 0.1 I, friction 0.6, dt=0.005).
These runs use shipped solver settings, including stagnation stopping, rather
than the frozen benchmark's tighter tolerances.

For 600 steps starting from geometric contact heights, mean solver sweeps:

| Cubes | Before | After |
|---|---:|---:|
| 1 | 90.81 | 2.43 |
| 2 | 52.41 | 2.71 |
| 4 | 278.12 | 8.55 |
| 8 | 571.77 | 20.48 |

The first 50-step release transient is nearly unchanged: four-cube peak speed
1.63556 -> 1.63592 m/s, eight-cube peak speed 3.49944 -> 3.50479 m/s. This is
not an initially equilibrated DSF stack, so the release motion is expected.

Settled diagnostics (400 settling steps, then 200 measured steps) show a
tradeoff: eight-cube maximum displacement increased from 10.10 to 16.72
micrometres and peak speed from 0.000279 to 0.002087 m/s. Fewer sweeps at the
default tolerance do not imply uniformly more accurate trajectories.

Contact state-regression literals were refreshed only after analytical tests
and comparisons with tighter solves. The regression test also runs those
tighter solves (tol_abs=1e-12, tol_rel=1e-13, stagnation disabled) and bounds
default-state errors for its fixtures by 5e-4; free-motion/kinematic references
are unchanged.

## Remaining direction

The residual-accounting and beta-ordering defects are addressed. The difficult
heterogeneous-chain case still fails the tight tolerance; mass-aware
scaling/preconditioning is a separate next experiment. Do not infer
a convergence rate from the old dual diagnostic, or extrapolate these frozen
fixtures to all stacking-apps scenes. Posegen's solver was not modified.
