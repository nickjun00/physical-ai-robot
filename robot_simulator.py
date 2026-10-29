"""Geometry and safety controls for the robot simulator.

The module deliberately has no Streamlit dependency so its safety behaviour can
be tested and reused by another UI or control loop.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Iterable, Optional, Tuple


EPSILON = 1e-9


@dataclass(frozen=True)
class Point:
    x: float
    y: float

    def distance_to(self, other: "Point") -> float:
        return hypot(self.x - other.x, self.y - other.y)


@dataclass(frozen=True)
class RectangleObstacle:
    """An axis-aligned, solid obstacle measured in metres."""

    name: str
    x: float
    y: float
    width: float
    height: float

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Obstacle width and height must be positive")

    def distance_to(self, point: Point) -> float:
        """Shortest distance from a point to this rectangle (zero when inside)."""
        dx = max(self.x - point.x, 0.0, point.x - (self.x + self.width))
        dy = max(self.y - point.y, 0.0, point.y - (self.y + self.height))
        return hypot(dx, dy)

    def expanded(self, clearance: float) -> "RectangleObstacle":
        """Conservatively expand the obstacle for clearance checking.

        The square expansion over-approximates a circular robot at corners,
        which may reject a tight diagonal route but cannot admit a collision.
        """
        if clearance < 0:
            raise ValueError("Clearance cannot be negative")
        return RectangleObstacle(
            self.name,
            self.x - clearance,
            self.y - clearance,
            self.width + 2 * clearance,
            self.height + 2 * clearance,
        )


@dataclass(frozen=True)
class ObstacleReading:
    name: str
    distance: float


@dataclass(frozen=True)
class MoveResult:
    requested: Point
    position: Point
    moved: bool
    blocked: bool
    obstacle: Optional[str] = None


def _contains(rectangle: RectangleObstacle, point: Point) -> bool:
    return (
        rectangle.x - EPSILON <= point.x <= rectangle.x + rectangle.width + EPSILON
        and rectangle.y - EPSILON <= point.y <= rectangle.y + rectangle.height + EPSILON
    )


def segment_rectangle_hit(
    start: Point, end: Point, rectangle: RectangleObstacle
) -> Optional[float]:
    """Return first segment fraction touching a rectangle, or ``None``.

    This slab-intersection test checks the whole commanded segment, unlike a
    fixed-rate position sample, so a long command cannot skip a thin obstacle.
    """
    if _contains(rectangle, start):
        return 0.0

    dx, dy = end.x - start.x, end.y - start.y
    enter, leave = 0.0, 1.0
    for origin, delta, lower, upper in (
        (start.x, dx, rectangle.x, rectangle.x + rectangle.width),
        (start.y, dy, rectangle.y, rectangle.y + rectangle.height),
    ):
        if abs(delta) <= EPSILON:
            if origin < lower or origin > upper:
                return None
            continue
        first, second = (lower - origin) / delta, (upper - origin) / delta
        if first > second:
            first, second = second, first
        enter, leave = max(enter, first), min(leave, second)
        if enter - leave > EPSILON:
            return None
    return max(0.0, enter) if leave >= -EPSILON and enter <= 1.0 + EPSILON else None


class RobotSimulator:
    """A circular robot that detects and safely stops before rectangular obstacles."""

    def __init__(
        self,
        obstacles: Iterable[RectangleObstacle] = (),
        *,
        position: Point = Point(0.0, 0.0),
        robot_radius: float = 0.20,
        safety_margin: float = 0.05,
    ) -> None:
        if robot_radius < 0 or safety_margin < 0:
            raise ValueError("Robot radius and safety margin cannot be negative")
        self.obstacles = tuple(obstacles)
        self.position = position
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin

    @property
    def clearance(self) -> float:
        return self.robot_radius + self.safety_margin

    def detect_obstacles(self, sensor_range: float) -> Tuple[ObstacleReading, ...]:
        """Return obstacles within range, measured from the robot's outer edge."""
        if sensor_range < 0:
            raise ValueError("Sensor range cannot be negative")
        readings = []
        for obstacle in self.obstacles:
            distance = max(0.0, obstacle.distance_to(self.position) - self.robot_radius)
            if distance <= sensor_range + EPSILON:
                readings.append(ObstacleReading(obstacle.name, distance))
        return tuple(sorted(readings, key=lambda reading: (reading.distance, reading.name)))

    def move_to(self, target: Point) -> MoveResult:
        """Move toward target, stopping before the first inflated obstacle hit."""
        start = self.position
        dx, dy = target.x - start.x, target.y - start.y
        length = hypot(dx, dy)
        hits = [
            (hit, obstacle)
            for obstacle in self.obstacles
            if (hit := segment_rectangle_hit(start, target, obstacle.expanded(self.clearance)))
            is not None
        ]
        if not hits:
            self.position = target
            return MoveResult(target, target, length > EPSILON, False)

        hit_fraction, obstacle = min(hits, key=lambda item: item[0])
        # Keep a tiny positive separation from the expanded obstacle.  The
        # safety margin is already represented by expansion; this is numerical
        # back-off so floating point contact never becomes penetration.
        safe_fraction = max(0.0, hit_fraction - (EPSILON / length if length else 0.0))
        safe_position = Point(start.x + dx * safe_fraction, start.y + dy * safe_fraction)
        self.position = safe_position
        return MoveResult(target, safe_position, safe_fraction > EPSILON, True, obstacle.name)
