# Design

## Context

See proposal.md for motivation. The repository has a configurable `FieldGeometry`, a coupled `simulate_flight` API that accepts a launch state, and a Matplotlib 3D trajectory animation. There is no tactical-scenario model or inverse launch solver. The existing default aerodynamic coefficients are provisional and must remain visible as a limitation in any probability result.

## Goals / Non-Goals

**Goals:**
- Keep pass planning, nominal launch solving, stochastic execution, and outcome evaluation as separate domain operations.
- Provide a small local top-down Matplotlib interaction for setting the throw origin, target, and nominal catch time.
- Make stochastic analysis reproducible with an explicit random generator seed.
- Allow a later game loop to request one concrete execution without mutating the planned pass.

**Non-Goals:**
- Calibrating throw variation from real athletes or claiming calibrated probabilities.
- Modeling receiver cuts, defender movement, force strategies, or autonomous players.
- Modeling pump fakes, pivot legality, or marker reactions.
- Saving, sharing, or synchronizing scenarios.
- Replacing the current flight model or validating its provisional aerodynamic coefficients.

## Decisions

1. **Separate the plan from execution.** Store the intended receiver location and catch time as immutable planning values. A nominal launch solution and sampled launches are derived results, not edits to that plan. This prevents analysis or a game simulation from moving the tactical target.

2. **Use a bounded shooting solver for the nominal release.** The selected throw profile supplies release attitude, spin convention, allowable speed range, and other fixed release constraints. Solve for the translational release velocity that minimizes position error at the requested catch time using repeated flight integrations and a bounded least-squares search. Accept a solution only when it meets a configured target tolerance and reaches the requested time without ground contact; otherwise return an explicit infeasible result. An approximate ballistic velocity is a suitable initial guess, not a successful answer by itself. Alternatives such as asking users to enter release velocity directly do not satisfy the agreed target-and-time planning workflow.

3. **Represent execution error in release coordinates.** Configure variation by throw type and distance using zero-mean truncated normal deviations for release speed, horizontal direction, vertical angle, and spin. Keep those spreads configurable and marked provisional until measured data is available. Convert each draw into a launch state and run the ordinary flight integrator; never perturb a solved landing point after integration. A supplied NumPy random generator makes both a Monte Carlo batch and a single execution repeatable.

4. **Make the initial success criterion a catch window.** At the nominal catch time, compare each flight with a configurable spatial catch envelope around the planned target and a configurable time tolerance. A disc that contacts the ground before the catch window is not catchable. Report successful-trial count, total trial count, and the resulting proportion. Do not call this a full pass-completion probability until receiver movement and defensive interception are modeled.

5. **Use Matplotlib as a thin local interaction layer.** Build a top-down field view from `FieldGeometry`. Mouse selection sets the throw origin and target; a small widget/input control sets nominal catch time and the throw profile. The view displays the nominal plan and whether a feasible launch was found, then allows the user to request the catchability estimate. Keep scenario and evaluation data usable without a GUI so they remain unit-testable.

6. **Keep defensive behavior and fakes composable but separate.** Later defense can add time-dependent reachable regions and policy decisions to the success evaluator without changing planned-pass or throw-variation data. A fake should be a timed thrower action that may change the marker's response, not an artificial displacement of the planned throw or an immediate random penalty. These behaviors are deferred until the first pass-evaluation workflow is established.

## Risks / Trade-offs

- **Repeated integration makes inverse solving and Monte Carlo costly** -> Bound solver evaluations, use a ballistic initial guess, expose trial count, and test runtime on representative short and long throws.
- **A bounded optimizer can fail or find a local solution** -> Validate the final trajectory against the target tolerance, report infeasibility explicitly, and test both reachable and unreachable plans.
- **Provisional aerodynamics can bias every estimate** -> Label the output as flight-level catchability and surface coefficient provenance and calibration status.
- **Uncalibrated throw spreads can create misleading precision** -> Mark profiles provisional, keep parameters configurable, and report the actual trial count rather than implying certainty.
- **Matplotlib event handling is harder to test than domain calculations** -> Keep callbacks thin and test scenario creation and evaluation independently; include a headless rendering test for the view.

## Migration Plan

No persisted scenarios or existing public tactical APIs need migration. Add the domain and UI alongside the current modules. Rollback consists of removing the new scenario/evaluation and editor modules and the corresponding documentation and tests; existing field and flight APIs remain available.
