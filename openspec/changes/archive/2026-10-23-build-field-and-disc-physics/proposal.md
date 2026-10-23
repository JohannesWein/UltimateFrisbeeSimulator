# Proposal

## Why

The simulator needs a trustworthy physical foundation before tactics or autonomous players can produce useful results. A precise, configurable field model and reproducible disc trajectories make later throw-window analysis possible while keeping the first step small enough to test independently.

## What Changes

- Introduce a field model with regulation-size defaults, explicit end zones, and configurable dimensions.
- Introduce a deterministic three-dimensional disc-flight simulation with gravity, aerodynamic lift and drag, spin, orientation changes, aerodynamic moments, and gyroscopic precession under configurable launch and wind conditions.
- Use the Discraft UltraStar as the named reference disc while keeping disc-specific parameter values traceable to sources or identified assumptions.
- Provide ordered, testable trajectory samples with position, linear velocity, orientation, and termination reason in consistent physical units.
- Add NumPy and SciPy as project dependencies for vector calculations and coupled differential-equation integration.
- Keep the model local and independent of UI, tactical decisions, and cloud services.

## Capabilities

### New Capabilities
- `field-geometry`: Represent a regulation Ultimate field, its playing area and end zones, and configurable dimensions.
- `disc-flight-physics`: Simulate reproducible disc trajectories with aerodynamic forces, spin, and gyroscopic precession from launch and environmental conditions.

### Modified Capabilities

None.

## Impact

The repository currently contains no application code, package manifest, or existing capability specs. Its `.gitignore` contains Python-specific rules, which suggests Python but does not establish a runtime or dependency policy. This change will establish a Python project and add a local simulation core, tests, and NumPy/SciPy dependencies. The Discraft UltraStar is the reference disc; a local copy of Koyanagi et al. (2012), which reports wind-tunnel force/moment measurements and flight-data validation, provides a candidate empirical source. Its tested disc and conditions must be verified before its coefficients are treated as UltraStar-specific. All other aerodynamic coefficients remain traceable and calibratable; the model will not use computational fluid dynamics.
