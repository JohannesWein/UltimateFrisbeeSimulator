# Spec Delta

## Purpose

Lets a tactical scenario describe an intended pass separately from imperfect throw execution. It estimates flight-level catchability over varied physical throws while preserving a nominal target and catch time for play design.

## ADDED Requirements

### Requirement: Define a planned pass in a field view
The system SHALL provide a top-down view of the configured field where a user can define the throw origin and intended receiver location for a planned pass.

#### Scenario: Set pass positions in the field view
- **WHEN** a user selects a throw origin and intended receiver location in the field view
- **THEN** the scenario stores both locations in field coordinates measured in metres
- **AND** the view displays the planned pass between them

#### Scenario: Set the nominal catch time
- **WHEN** a user edits the nominal catch time for a planned pass
- **THEN** the displayed and evaluated pass plan uses that time without changing the intended receiver location

### Requirement: Find a feasible nominal release
The system SHALL find a nominal launch state whose simulated flight reaches the planned receiver location at the nominal catch time, subject to the selected throw profile's release constraints.

#### Scenario: Solve an achievable planned pass
- **WHEN** a planned target and catch time are reachable within the selected throw profile's release constraints
- **THEN** the system returns a nominal launch state whose simulated disc position is within the configured target tolerance at the nominal catch time

#### Scenario: Report an unreachable planned pass
- **WHEN** no launch state within the selected throw profile's release constraints reaches the target at the nominal catch time
- **THEN** the system reports that no feasible nominal throw was found rather than presenting an unsolved trajectory as a valid plan

### Requirement: Preserve the nominal planned pass
The system SHALL represent a planned pass with its intended receiver location and nominal catch time independently of any sampled throw execution.

#### Scenario: Create a planned pass
- **WHEN** a user defines a pass in a tactical scenario
- **THEN** the plan retains its intended receiver location and nominal catch time as the stable tactical target
- **AND** evaluating or executing the pass does not overwrite those planned values

#### Scenario: Reject invalid pass targets
- **WHEN** a planned pass has non-finite coordinates or a non-positive nominal catch time
- **THEN** the system reports an explicit validation error

### Requirement: Configure throw execution variation
The system SHALL apply configurable variation to physical launch parameters according to throw type and throw distance, rather than perturbing the final landing location directly.

#### Scenario: Vary a planned throw
- **WHEN** a planned pass is evaluated with a configured throw-variation profile
- **THEN** each trial varies the configured launch parameters around the nominal release
- **AND** each varied execution produces its own flight through the flight-physics model

#### Scenario: Keep throw variation distinct from model uncertainty
- **WHEN** a completion estimate is presented
- **THEN** execution variation is identified separately from provisional aerodynamic coefficients and other physical-model assumptions

### Requirement: Estimate flight-level catchability
The system SHALL estimate how often varied throw executions reach an explicit receiver catch window without implying unmodeled player or defensive behavior.

#### Scenario: Estimate completion over repeated trials
- **WHEN** a planned pass is evaluated with a valid trial count, random seed, throw profile, and success criteria
- **THEN** the result reports the fraction of trials that reach the receiver's configured catch window and the total number of trials
- **AND** repeating the evaluation with identical inputs and seed produces identical trial outcomes and estimate

#### Scenario: Distinguish flight catchability from full pass completion
- **WHEN** the scenario has no modeled receiver movement or defensive interception behavior
- **THEN** the estimate is identified as flight-level catchability and is not presented as a full pass-completion probability

#### Scenario: Report unsuccessful or invalid evaluation
- **WHEN** an evaluation configuration is invalid or a trial cannot produce a valid flight
- **THEN** the system reports the error explicitly rather than silently treating that trial as a success

### Requirement: Execute one concrete throw without changing its plan
The system SHALL support sampling one concrete throw execution from a planned pass for use by a later game simulation.

#### Scenario: Execute a sampled throw
- **WHEN** a game simulation executes a planned pass with a supplied random generator state
- **THEN** it produces one concrete launch state and corresponding flight trajectory
- **AND** the planned receiver location and nominal catch time remain unchanged

#### Scenario: Repeat a sampled execution
- **WHEN** the same planned pass and random generator state are used again
- **THEN** the sampled launch state and trajectory are reproducible
