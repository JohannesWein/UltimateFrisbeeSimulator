"""SI field geometry and position classification."""

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real


class FieldRegion(Enum):
    """A field position's region along the field's length."""

    OUTSIDE = "outside"
    END_ZONE_A = "end_zone_a"
    PLAYING_AREA = "playing_area"
    END_ZONE_B = "end_zone_b"


@dataclass(frozen=True)
class FieldGeometry:
    """Rectangular Ultimate field measured in metres.

    The origin is the corner at the start of end zone A. The x-axis runs
    along the field's length and the y-axis across its width.
    """

    length_m: float = 100.0
    width_m: float = 37.0
    end_zone_depth_m: float = 18.0

    def __post_init__(self) -> None:
        for name, value in (
            ("length_m", self.length_m),
            ("width_m", self.width_m),
            ("end_zone_depth_m", self.end_zone_depth_m),
        ):
            if isinstance(value, bool) or not isinstance(value, Real):
                raise ValueError(f"{name} must be a finite positive number")
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a finite positive number")

        if 2 * self.end_zone_depth_m >= self.length_m:
            raise ValueError(
                "end zones must leave a positive-length central playing area"
            )

    @property
    def playing_area_length_m(self) -> float:
        """Length of the central playing area, excluding both end zones."""
        return self.length_m - 2 * self.end_zone_depth_m

    def classify_position(self, x_m: float, y_m: float) -> FieldRegion:
        """Classify a 2D position; goal-line boundaries belong to end zones."""
        if (
            isinstance(x_m, bool)
            or isinstance(y_m, bool)
            or not isinstance(x_m, Real)
            or not isinstance(y_m, Real)
            or not math.isfinite(x_m)
            or not math.isfinite(y_m)
        ):
            raise ValueError("position coordinates must be finite numbers")

        if not (0 <= x_m <= self.length_m and 0 <= y_m <= self.width_m):
            return FieldRegion.OUTSIDE
        if x_m <= self.end_zone_depth_m:
            return FieldRegion.END_ZONE_A
        if x_m >= self.length_m - self.end_zone_depth_m:
            return FieldRegion.END_ZONE_B
        return FieldRegion.PLAYING_AREA
