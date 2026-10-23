import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter

from ultimate_frisbee_simulator import (
    FlightInitialState,
    SolverSettings,
    animate_flight,
    discraft_ultra_star,
    simulate_flight,
)


disc = discraft_ultra_star()
trajectory = simulate_flight(
    disc=disc,
    initial_state=FlightInitialState(
        position_m=(32.0, 18.5, 1.8),
        velocity_m_s=(12.0, 0.0, 3.0),
        angular_velocity_body_rad_s=(0.0, 0.0, 50.0),
    ),
    settings=SolverSettings(maximum_duration_s=4.0),
)

parser = argparse.ArgumentParser(description="Animate an Ultimate Frisbee disc flight.")
parser.add_argument(
    "--save",
    type=Path,
    help="save the animation as a GIF instead of opening a window",
)
arguments = parser.parse_args()

animation = animate_flight(trajectory, disc)
if arguments.save:
    animation.save(arguments.save, writer=PillowWriter(fps=30))
else:
    plt.show()
