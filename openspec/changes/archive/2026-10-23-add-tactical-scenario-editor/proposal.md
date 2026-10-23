# Proposal

## Why

The simulator can calculate and visualize a disc flight, but it cannot yet represent a tactical pass as a planned action in a field scenario. A useful coaching tool must distinguish the intended pass from the imperfect throw that a person actually executes, and estimate outcomes without pretending that every throw is identical.

## What Changes

- Add a local, top-down Matplotlib scenario view based on the existing field geometry for setting a thrower, intended receiver location, and throw profile.
- Make target-only planning the default: find a nominal release and catch time without requiring the user to prescribe flight duration.
- Retain an optional timing-pass mode that finds a nominal release for a user-specified catch time.
- Represent a resolved planned pass by its intended receiver location and nominal catch time, preserving both independently of execution.
- Distinguish failure to find a verified nominal release from proof that a pass is physically impossible.
- Add configurable throw-execution variation by throw type and distance, applied to launch parameters and propagated through the existing flight simulation.
- Estimate the probability that repeated, reproducible throw trials reach the receiver's catch window; distinguish this flight-level estimate from a full pass-completion probability until player and defensive behavior is modeled.
- Preserve the nominal plan for play design while allowing a later game simulation to sample one concrete execution for each throw.
- Keep player-specific calibration, autonomous movement, defensive policies, and learned agents out of scope for this change.

## Capabilities

### New Capabilities

- `throw-evaluation`: Represent planned passes separately from their varied execution and estimate outcomes over reproducible trials.

### Modified Capabilities

None.

## Impact

The change will add a local Matplotlib scenario workflow and throw-evaluation APIs alongside the existing field, flight, and visualization modules. It will use the current SciPy flight integrator, add no cloud service, and keep throw variation parameters configurable so they can later be calibrated from measured throws. The default aerodynamic coefficients remain provisional; estimated probabilities must identify that model limitation.
