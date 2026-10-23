# Tasks

## 1. Planned Pass and Nominal Release

- [x] 1.1 Add validated planned-pass and throw-profile models that keep target location and catch time separate from execution; verify unit tests cover valid values and invalid coordinates/times.
- [x] 1.2 Implement bounded nominal-release solving against the target at the requested catch time, including exact-time trajectory evaluation and explicit infeasible results; verify tests cover reachable, out-of-tolerance, and ground-before-catch cases.
- [x] 1.3 Document the nominal pass and release-solving API, constraints, and limitations in the README; verify the documented example finds a feasible throw.

## 2. Throw Variation and Evaluation

- [x] 2.1 Implement configurable throw-type and distance variation for release speed, direction, elevation, and spin; verify tests cover bounded sampling, profile validation, and repeatability with a fixed random generator.
- [x] 2.2 Implement seeded Monte Carlo evaluation against a configurable spatial/time catch window and report successful-trial count, trial count, and flight-model provenance; verify identical seeds reproduce outcomes and invalid trials are reported explicitly.
- [x] 2.3 Document variation parameters, provisional calibration status, success criteria, and the distinction between flight-level catchability and full pass completion; verify the documented evaluation example runs.

## 3. Top-Down Scenario View

- [x] 3.1 Add a Matplotlib field editor for setting throw origin, target, nominal catch time, and throw profile; verify mouse/widget interactions update the scenario in field coordinates.
- [x] 3.2 Display the planned pass, feasible nominal trajectory, and flight-level catchability result; verify a headless rendering test exercises the view and invalid plans are visibly identified.
- [x] 3.3 Document how to launch and use the scenario editor locally; verify the documented command starts without requiring a cloud service.

## 4. Integration Verification

- [x] 4.1 Add an end-to-end test that creates a planned pass, solves its nominal release, evaluates seeded execution variations, and renders the field view; verify the full test suite passes with `uv run python -m unittest discover -v`.

## 5. Target-Only Planning and Optional Timing Passes

The completed tasks above describe the initial fixed-time implementation. The following tasks implement the revised planning modes.

- [x] 5.1 Add validated target-only planning requests and automatic timing search settings while preserving existing fixed-time planned-pass APIs; test positive finite time bounds, ordered limits, valid budgets, and resolved-plan immutability.
- [x] 5.2 Implement deterministic, budgeted automatic timing search using the existing fixed-time nominal-release solver; select the earliest verified catch time among examined candidates and test exact-time target tolerance, no prior ground contact, reproducibility, budget enforcement, and explicit unresolved results.
- [x] 5.3 Distinguish search failure from physical impossibility in solver diagnostics and user-visible messages; test that ground-before-catch describes the failed candidate and does not assert that all possible releases fail.
- [x] 5.4 Make target-only planning the editor default and retain optional timing-pass selection; display derived catch time in automatic mode, enable fixed-time input only in timing mode, and clear stale results when inputs or mode change; verify widget interactions and headless rendering for both modes.
- [x] 5.5 Connect both planning modes to catchability evaluation and single-execution sampling through the resolved nominal plan; test that target and resolved catch time stay unchanged and unresolved plans cannot be evaluated.
- [x] 5.6 Update the README with both planning workflows, automatic search bounds and selection limitations, and an executable example for each mode; verify the examples run.
- [x] 5.7 Add end-to-end regression coverage for automatic planning, explicit timing passes without time substitution, mode switching, and failed-search feedback; verify the full suite passes with `uv run python -m unittest discover -v`.
