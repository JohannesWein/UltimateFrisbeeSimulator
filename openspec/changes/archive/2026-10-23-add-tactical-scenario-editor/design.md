# Design

## Context

See proposal.md for motivation. The repository has a configurable `FieldGeometry`, a coupled `simulate_flight` API that accepts a launch state, and a Matplotlib 3D trajectory animation. The initial scenario editor, fixed-time nominal-release solver, and seeded catchability evaluation are implemented. This revision adds default target-only planning while retaining explicit timing passes. The existing default aerodynamic coefficients are provisional and must remain visible as a limitation in any probability result.

## Goals / Non-Goals

**Goals:**
- Keep pass planning, nominal launch solving, stochastic execution, and outcome evaluation as separate domain operations.
- Provide a small local top-down Matplotlib interaction for setting the throw origin, target, and throw profile, with automatic timing by default and an optional fixed-time mode.
- Make stochastic analysis reproducible with an explicit random generator seed.
- Allow a later game loop to request one concrete execution without mutating the planned pass.

**Non-Goals:**
- Calibrating throw variation from real athletes or claiming calibrated probabilities.
- Modeling receiver cuts, defender movement, force strategies, or autonomous players.
- Modeling pump fakes, pivot legality, or marker reactions.
- Saving, sharing, or synchronizing scenarios.
- Replacing the current flight model or validating its provisional aerodynamic coefficients.
- Expanding release-attitude constraints or claiming a globally optimal throw.

## Decisions

1. **Separate planning requests, resolved plans, and execution.** A target-only request contains the throw origin, intended receiver location, and selected throw profile without a prescribed catch time. A timing-pass request additionally constrains the catch time. After successful planning, store the target and nominal catch time as immutable resolved planning values. A nominal launch solution and sampled launches do not change those values. Reuse the existing fixed-time planned-pass representation for resolved plans and preserve its API for timing-pass callers.

2. **Reuse bounded fixed-time solving inside automatic timing search.** The selected throw profile supplies release attitude, spin convention, allowable speed range, and other fixed release constraints. For a specified candidate catch time, solve for translational release velocity using repeated flight integrations and bounded least squares. Accept a solution only when the verified exact-time position meets the configured target tolerance without prior ground contact. An approximate ballistic velocity is a starting guess, not a successful answer. Target-only planning examines a deterministic, bounded set of candidate times within configurable positive time limits and explicit search budgets. Select the earliest verified catch time among the examined candidates; do not claim a continuous global minimum or optimize stochastic catchability during planning. A timing pass uses only its requested time, never an automatic substitute. If the search finds no verified candidate, return an explicit search failure. Early ground contact describes the final failed candidate, not proof that every permitted release is impossible.

3. **Represent execution error in release coordinates.** Configure variation by throw type and distance using zero-mean truncated normal deviations for release speed, horizontal direction, vertical angle, and spin. Keep those spreads configurable and marked provisional until measured data is available. Convert each draw into a launch state and run the ordinary flight integrator; never perturb a solved landing point after integration. A supplied NumPy random generator makes both a Monte Carlo batch and a single execution repeatable.

4. **Make the initial success criterion a catch window.** At the nominal catch time, compare each flight with a configurable spatial catch envelope around the planned target and a configurable time tolerance. A disc that contacts the ground before the catch window is not catchable. Report successful-trial count, total trial count, and the resulting proportion. Do not call this a full pass-completion probability until receiver movement and defensive interception are modeled.

5. **Use Matplotlib as a thin local interaction layer.** Build a top-down field view from `FieldGeometry`. Mouse selection sets the throw origin and target; widgets select the throw profile and planning mode. Target-only planning is the default and shows the derived catch time as a result, with no active fixed-time input. Timing-pass mode enables the user-specified catch-time input. Mode, target, profile, or timing changes invalidate previous release and catchability results. The view displays the resolved nominal plan and verified trajectory, then evaluates catchability around that fixed nominal time. Failed searches remain visibly unresolved and cannot trigger Monte Carlo evaluation. Keep requests, resolved plans, and evaluation usable without a GUI.

6. **Keep defensive behavior and fakes composable but separate.** Later defense can add time-dependent reachable regions and policy decisions to the success evaluator without changing planned-pass or throw-variation data. A fake should be a timed thrower action that may change the marker's response, not an artificial displacement of the planned throw or an immediate random penalty. These behaviors are deferred until the first pass-evaluation workflow is established.

## Risks / Trade-offs

- **Repeated integration makes inverse solving and Monte Carlo costly** -> Bound solver evaluations, use a ballistic initial guess, expose trial count, and test runtime on representative short and long throws.
- **Automatic timing multiplies inverse-solving cost** -> Bound both the number of candidate times and per-candidate optimization work, use deterministic ordering, and test that configured budgets are respected.
- **A bounded optimizer can fail or find a local solution** -> Validate the final trajectory against the target tolerance, report infeasibility explicitly, and test both reachable and unreachable plans.
- **The earliest examined solution is not necessarily the earliest possible solution** -> Document the finite search policy and distinguish search failure from physical impossibility.
- **Provisional aerodynamics can bias every estimate** -> Label the output as flight-level catchability and surface coefficient provenance and calibration status.
- **Uncalibrated throw spreads can create misleading precision** -> Mark profiles provisional, keep parameters configurable, and report the actual trial count rather than implying certainty.
- **Matplotlib event handling is harder to test than domain calculations** -> Keep callbacks thin and test scenario creation and evaluation independently; include a headless rendering test for the view.

## Migration Plan

No persisted scenarios need migration. Preserve the existing fixed-time planning and evaluation APIs. Add target-only resolution alongside them, and intentionally change the editor default to automatic timing while retaining explicit timing-pass selection. Existing completed tasks describe the initial fixed-time implementation; appended open tasks track this revision. Rollback of this revision restores the fixed-time editor default without removing existing field, flight, variation, or evaluation APIs.
