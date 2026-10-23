import unittest

from ultimate_frisbee_simulator import (
    FieldGeometry,
    FieldRegion,
    FlightInitialState,
    FlightTerminationReason,
    SolverSettings,
    discraft_ultra_star,
    simulate_flight,
)


class SimulatorSmokeTests(unittest.TestCase):
    def test_flight_landing_can_be_classified_on_the_field(self):
        field = FieldGeometry()
        trajectory = simulate_flight(
            disc=discraft_ultra_star(),
            initial_state=FlightInitialState(
                position_m=(50.0, 18.5, 1.8),
                velocity_m_s=(12.0, 0.0, 3.0),
                angular_velocity_body_rad_s=(0.0, 0.0, 50.0),
            ),
            settings=SolverSettings(maximum_duration_s=4.0),
        )
        final_sample = trajectory.samples[-1]
        landing_region = field.classify_position(
            final_sample.position_m[0],
            final_sample.position_m[1],
        )

        self.assertIs(
            trajectory.termination_reason,
            FlightTerminationReason.GROUND_CONTACT,
        )
        self.assertIs(landing_region, FieldRegion.PLAYING_AREA)


if __name__ == "__main__":
    unittest.main()
