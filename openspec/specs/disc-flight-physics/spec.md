# disc-flight-physics Specification

## Purpose

Provides reproducible, physically based disc trajectories including aerodynamic forces, rotational dynamics, and gyroscopic precession from explicit launch and environmental conditions.

## Requirements

### Requirement: Simulate a three-dimensional disc trajectory with rotation and precession
The system SHALL calculate a disc trajectory in metres and seconds from its launch position, linear velocity, rotational velocity, attitude, physical parameters, and environmental conditions.

#### Scenario: Simulate a curved throw with spin
- **WHEN** a throw is simulated with valid launch velocity, spin rate, and orientation
- **THEN** the result contains ordered samples with elapsed time, three-dimensional position, linear velocity, and orientation
- **AND** the trajectory accounts for gravity, aerodynamic drag, lift, and aerodynamic moments that produce gyroscopic precession and curved flight

#### Scenario: Apply environmental wind
- **WHEN** a throw is simulated with a non-zero wind vector
- **THEN** aerodynamic forces and relative airflow are calculated from disc motion relative to the moving airmass

### Requirement: Provide a traceable Discraft UltraStar reference profile
The system SHALL support the Discraft UltraStar as a named reference disc and SHALL keep its physical and aerodynamic parameters explicit, configurable, and traceable to documented sources or identified assumptions.

#### Scenario: Select the UltraStar reference disc
- **WHEN** a simulation selects the Discraft UltraStar reference profile
- **THEN** the profile identifies the disc model and supplies its configured physical and aerodynamic parameters
- **AND** each parameter records its source or is explicitly marked as an assumption requiring measurement or calibration

#### Scenario: Use data from a different disc
- **WHEN** published or measured aerodynamic data is considered for the UltraStar profile and the tested disc or conditions are not established as equivalent
- **THEN** the data is identified as comparative or provisional rather than represented as UltraStar-specific

### Requirement: Produce deterministic and bounded trajectories
The system SHALL produce the same trajectory for identical inputs and SHALL end a simulation at ground contact or its configured maximum duration.

#### Scenario: Repeat a simulation
- **WHEN** the same throw inputs and numerical settings are simulated more than once
- **THEN** each run produces identical trajectory samples

#### Scenario: End a flight
- **WHEN** a simulated disc reaches ground level or the configured maximum duration
- **THEN** the trajectory stops and reports which condition ended the simulation

### Requirement: Validate simulation inputs
The system SHALL report invalid launch, physical, environmental, or numerical parameters as explicit validation errors rather than returning a successful trajectory.

#### Scenario: Reject invalid simulation parameters
- **WHEN** any simulation parameter is non-finite, mass, reference area, air density, or a principal moment of inertia is non-positive, or an integration tolerance, maximum integration step, or maximum duration is non-positive
- **THEN** simulation fails with an explicit validation error
