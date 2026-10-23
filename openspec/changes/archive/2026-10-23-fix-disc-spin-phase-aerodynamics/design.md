# Design

## Context

See [proposal.md](proposal.md) for motivation. `evaluate_aerodynamics` currently computes lift in the rotating body x/z plane, evaluates sideslip-dependent surfaces from rotating body y, and applies moment components directly about rotating material axes. All three introduce spin-phase dependence. Merely fixing the lift direction would leave phase-dependent drag and torque.

`simulate_flight` integrates a body-to-world quaternion and body angular velocity with Euler's rigid-body equation. It already transforms body forces to world coordinates and supports exact-time samples. The UltraStar profile has equal in-plane principal moments. Preserve this integrator and its public body-frame wrench contract.

The editor uses bounded fixed-time least squares within a finite automatic timing grid. Its default release attitude is horizontal; it does not search nose angle or bank. The completed editor change explicitly excluded aerodynamic corrections. This separate change intentionally changes the physics it consumes.

## Goals / Non-Goals

**Goals:**
- Remove material spin phase from all aerodynamic load directions and coefficient inputs.
- Preserve wind, inclination, spin-rate/direction response, and coupled precession.
- Make geometric conventions, singular cases, compatibility effects, and acceptance thresholds explicit.
- Recheck the screenshot pass using the corrected ordinary solver and unchanged search budgets.

**Non-Goals:**
- Calibrating coefficients against measurements or claiming real-world pass probabilities.
- Adding asymmetric discs, a second legacy aerodynamic mode, spin-dependent empirical lift laws, or new dependencies.
- Expanding the nominal solver to attitude/spin optimization or changing its grid, budgets, timing substitution policy, or success tolerances.
- Updating defense, fakes, movement, or the GUI layout.

## Decisions

### 1. Use a flow-aligned frame, then return a body-frame wrench

For body-frame wind-relative translational velocity `v`, disc normal `n=(0,0,1)`, and nondegenerate in-plane velocity:

- `u = v / |v|`
- `x_flow = normalize(v - dot(v,n)*n)`
- `y_flow = cross(n,x_flow)`
- `z_flow = n`
- positive-lift direction `normalize(n - dot(n,u)*u)`
- drag direction `-u`

Evaluate coefficient magnitudes using total relative speed and transform flow-frame side-force and roll/pitch/yaw moments into body coordinates through these axes. The existing integrator then performs the ordinary body-to-world transformation. Roll, pitch, and yaw refer to flow x, flow y, and disc normal respectively; they are not fixed to printed material marks.

This makes the complete world-space wrench invariant without dropping torques. It also works for arbitrary normal directions and wind. Alternatives rejected: freezing quaternion yaw would remove real spin; correcting lift alone leaves drag and moments wrong; removing all aerodynamic moments removes precession.

### 2. Keep surface data shapes, change their physical interpretation explicitly

Retain `CoefficientSurface`, `AerodynamicCoefficientModel`, and `AerodynamicWrench` field names, including `force_body_n`, `moment_body_nm`, and the actual body-frame relative velocity diagnostic.

For the axisymmetric model, evaluate all coefficient surfaces at sideslip zero and report aerodynamic sideslip zero. The two-dimensional surface interpolation API remains usable directly; the flight evaluator no longer samples its nonzero-sideslip columns. Explain this intentional compatibility change for custom coefficient models; do not silently claim old directional profiles retain identical behavior.

Use signed angle of attack `atan2(-dot(v,n), |v_in_plane|)`: nose-up inclination relative to the trajectory is positive. The old normal-velocity sign gave positive attack to a horizontal disc ascending through air; correct this geometric sign as part of defining the flow frame, with explicit nose-up/nose-down unit tests. Retain coefficient table values, identify them as provisional, and document the changed interpretation. Do not flip or tune table entries to reproduce old trajectories.

Alternative rejected: deriving an arbitrary sideslip from rotating body axes retains the defect. Adding an alternate legacy model solely for this bug would increase scope and preserve an invalid default.

### 3. Define singular flow without a material-axis fallback

With zero relative speed, return zero force and moment and finite zero angle diagnostics, as today. With nonzero speed and in-plane speed at or below `1e-12 * total_speed`, there is no unique flow x/y direction. Apply only axial drag and the coefficient-defined axial yaw moment; set transverse lift, side force, roll, and pitch contributions to zero. Attack tends to the appropriate signed right angle; sideslip remains zero.

This is an explicitly provisional degenerate-flow policy, not a high-angle calibration. It avoids division by zero and arbitrary body-x fallback. Test exact and near-normal flow, both signs, both sides of the threshold, and phase equivalence; do not assert continuity of uncalibrated transverse loads at normal incidence.

### 4. Verify symmetry in physical observables, not quaternion components

For force-level tests, sweep phases 0, 45, 90, 180, and 270 degrees for horizontal and tilted normals, with still air and wind. Include custom nonzero coefficients for each force/moment channel so zero defaults cannot hide a missing transformation. Require absolute world-force/moment differences no larger than 1e-9 N/Nm and equal attack angles.

For trajectory tests use the axisymmetric UltraStar inertia and phase-equivalent initial conditions: compose the original quaternion with a rotation about local z, and transform any transverse angular-velocity components so world angular velocity is unchanged. Compare requested samples at 0.1, 0.3, and 0.5 s from a sufficiently high release to avoid early termination:

- world position and velocity: maximum vector difference 1e-4 m and 1e-4 m/s;
- world disc normal: vector difference 1e-5;
- world angular velocity: vector difference 1e-3 rad/s.

Use the repository's standard strict integration tolerances. Quaternion components and adaptive solver sample counts need not match between different initial phases. Identical inputs must still reproduce identical output. If tolerances fail, investigate transformations or numerical convergence rather than weakening the assertions.

Add controlled nonzero pitching-torque fixtures comparing opposite spin directions and different spin magnitudes. Verify nonzero normal evolution and distinct angular-momentum/precession responses without requiring every launch to curve or hit the ground before a particular duration.

### 5. Treat the reported pass as an explicit end-to-end investigation gate

Use approximate screenshot coordinates `(36.5,17.0)` to `(63.0,26.5)` (28.15 m) with the default backhand, default reference disc, standard heights, still air, and unchanged automatic timing settings. These coordinates are an approximation, not recovered exact UI input. Also retain the existing short-pass and fixed-time examples.

Run the ordinary planner, independently verify any returned exact-time catch sample and no prior ground contact, then exercise resolved-plan sampling/catchability and headless editor rendering. A success must meet the existing 0.5 m nominal target tolerance; neither a ballistic fallback nor relaxed tolerance counts. Record runtime and attempted times without imposing an unmeasured performance target.

The anticipated outcome is a verified pass, but correcting phase dependence alone is not evidence that default attitude, coefficients, and local search can resolve it. If it remains unresolved, isolate the remaining cause with bounded forward-flight or solver diagnostics, report the evidence, and ask the user before adding calibration, attitude search, or budget changes. Do not mark the pass-regression task or whole change complete while this acceptance gate is unresolved.

## Risks / Trade-offs

- **Existing trajectories and optimizer results will change** -> Keep public data shapes, document physical-semantic changes, and update assertions only for demonstrated corrected behavior, not to suppress failures.
- **Provisional coefficients may remain unrealistic** -> Keep provenance visible in results and documentation; separate structural correctness from measured validation.
- **Normal-incidence flow has no unique transverse basis** -> Use the explicit axis-only policy and finite-value/symmetry tests rather than an arbitrary material axis.
- **Generic unequal in-plane inertias need not be phase-equivalent** -> Limit trajectory symmetry claims to axisymmetric inertia; do not reject custom inertia profiles or erase their actual dynamics.
- **The 28 m pass may reveal another limitation** -> Use the acceptance gate above and obtain scope approval rather than silently broadening this change.
- **Repeated inverse solves remain costly** -> Reuse existing bounded budgets and exact-time verification; record runtime without conflating correctness with speed.

## Migration Plan

1. Add failing load-invariance and flow-convention tests that reproduce the current defect.
2. Correct the evaluator without changing quaternion dynamics or coefficient table values.
3. Verify torque coupling, phase-equivalent trajectories, and existing domain/UI regressions.
4. Resolve the reported-pass acceptance gate and update README conventions and executable examples.
5. Run the full test suite, package build, diff checks, and strict OpenSpec validation.

No persisted scenario migration or new packages are required. Rollback is an ordinary revert of this change, including its tests and documentation, not an attempt to retain the faulty force law behind a runtime option.
