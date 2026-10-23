from pathlib import Path
import tempfile
import unittest

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from ultimate_frisbee_simulator import (
    FlightTerminationReason,
    FlightTrajectory,
    TrajectorySample,
    animate_flight,
    discraft_ultra_star,
)


class VisualizationTests(unittest.TestCase):
    def setUp(self):
        self.trajectory = FlightTrajectory(
            samples=(
                TrajectorySample(
                    elapsed_s=0.0,
                    position_m=(10.0, 12.0, 1.5),
                    velocity_m_s=(8.0, 0.0, 2.0),
                    orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
                    angular_velocity_body_rad_s=(0.0, 0.0, 20.0),
                ),
                TrajectorySample(
                    elapsed_s=0.1,
                    position_m=(10.8, 12.0, 1.65),
                    velocity_m_s=(8.0, 0.0, 1.0),
                    orientation_wxyz=(0.999, 0.0, 0.045, 0.0),
                    angular_velocity_body_rad_s=(0.0, 1.0, 20.0),
                ),
            ),
            termination_reason=FlightTerminationReason.MAXIMUM_DURATION,
        )

    def test_creates_and_renders_3d_animation(self):
        animation = animate_flight(
            self.trajectory,
            discraft_ultra_star(),
            frames_per_second=10,
        )
        figure = plt.gcf()
        try:
            self.assertIsInstance(animation, FuncAnimation)
            self.assertEqual(len(figure.axes), 1)
            self.assertEqual(figure.axes[0].name, "3d")

            with tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / "flight.gif"
                animation.save(output, writer=PillowWriter(fps=10))
                self.assertGreater(output.stat().st_size, 0)
        finally:
            plt.close(figure)

    def test_rejects_empty_trajectory_and_invalid_animation_settings(self):
        empty = FlightTrajectory((), FlightTerminationReason.MAXIMUM_DURATION)

        with self.assertRaisesRegex(ValueError, "at least one sample"):
            animate_flight(empty, discraft_ultra_star())
        with self.assertRaisesRegex(ValueError, "positive integer"):
            animate_flight(
                self.trajectory,
                discraft_ultra_star(),
                frames_per_second=0,
            )
        with self.assertRaisesRegex(ValueError, "trail_duration_s"):
            animate_flight(
                self.trajectory,
                discraft_ultra_star(),
                trail_duration_s=0,
            )

    def test_rejects_unsorted_trajectory_samples(self):
        unsorted = FlightTrajectory(
            samples=(self.trajectory.samples[1], self.trajectory.samples[0]),
            termination_reason=FlightTerminationReason.MAXIMUM_DURATION,
        )

        with self.assertRaisesRegex(ValueError, "finite, ordered"):
            animate_flight(unsorted, discraft_ultra_star())


if __name__ == "__main__":
    unittest.main()
