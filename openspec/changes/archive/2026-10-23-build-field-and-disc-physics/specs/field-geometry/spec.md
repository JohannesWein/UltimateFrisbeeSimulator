# Spec Delta

## Purpose

Provides a consistent, configurable coordinate model of an Ultimate field so simulation components can locate positions and distinguish playing areas.

## ADDED Requirements

### Requirement: Regulation field geometry
The system SHALL provide a field model with configurable length, width, and end-zone depth, defaulting to 100 m, 37 m, and 18 m respectively.

#### Scenario: Create a field with regulation defaults
- **WHEN** a field is created without custom dimensions
- **THEN** its length is 100 m, its width is 37 m, and each end zone is 18 m deep
- **AND** the central playing area is 64 m long

#### Scenario: Create a field with custom dimensions
- **WHEN** a field is created with valid custom length, width, and end-zone depth
- **THEN** its geometry uses those dimensions and derives the central playing-area length from them

### Requirement: Field coordinates and region classification
The system SHALL represent field positions in metres on a two-dimensional coordinate plane and classify positions as in the field's central playing area, either end zone, or outside the field.

#### Scenario: Classify positions in each field region
- **WHEN** positions are queried in the central playing area, either end zone, and beyond the field boundary
- **THEN** each position is classified in its corresponding region

#### Scenario: Keep region classification consistent at goal lines
- **WHEN** a position lies on an end-zone boundary
- **THEN** it is classified as part of that end zone

### Requirement: Validate field dimensions
The system SHALL reject dimensions that are non-positive or whose two end zones leave no positive-length central playing area.

#### Scenario: Reject invalid dimensions
- **WHEN** a field is configured with a non-positive length or width, a non-positive end-zone depth, or an end-zone depth at least half the total length
- **THEN** field creation fails with an explicit validation error
