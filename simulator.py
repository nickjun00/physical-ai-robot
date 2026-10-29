"""2D geometric simulation. Distances are metres; touching is unsafe."""
from dataclasses import dataclass
from math import hypot, isfinite


def finite(*values):
    if not all(isfinite(value) for value in values):
        raise ValueError("Coordinates and dimensions must be finite")


@dataclass(frozen=True)
class Point:
    x: float
    y: float

    def __post_init__(self):
        finite(self.x, self.y)


@dataclass(frozen=True)
class Obstacle:
    center: Point
    radius: float

    def __post_init__(self):
        finite(self.radius)
        if self.radius <= 0:
            raise ValueError("Obstacle radius must be positive")


def segment_distance(point, start, end):
    """Shortest distance to a closed segment, including its endpoints."""
    dx, dy = end.x - start.x, end.y - start.y
    length = hypot(dx, dy)
    if length == 0:
        return hypot(point.x - start.x, point.y - start.y)
    ux, uy = dx / length, dy / length
    projection = (point.x - start.x) * ux + (point.y - start.y) * uy
    distance = max(0.0, min(length, projection))
    return hypot(point.x - (start.x + distance * ux),
                 point.y - (start.y + distance * uy))


class Simulator:
    """Check a swept circular footprint before committing each straight move.

    Sensor range limits reported detections, never collision checks. Stops are
    latched until reset. Obstacles are static, known geometry, not noisy sensors.
    """

    def __init__(self, width=10.0, height=10.0, position=Point(1, 1),
                 robot_radius=0.25, safety_margin=0.1, sensor_range=2.0,
                 obstacles=()):
        finite(width, height, robot_radius, safety_margin, sensor_range)
        if min(width, height, robot_radius, sensor_range) <= 0 or safety_margin < 0:
            raise ValueError("Dimensions and range must be positive; margin nonnegative")
        self.width, self.height = width, height
        self.robot_radius, self.safety_margin = robot_radius, safety_margin
        self.sensor_range = sensor_range
        self.obstacles = tuple(obstacles)
        self.position = self._initial_position = position
        self.status, self.message = "IDLE", "Ready"
        if self._blocked(position, position):
            raise ValueError("Initial position violates obstacle or wall clearance")

    def _blocked(self, start, end):
        envelope = self.robot_radius + self.safety_margin
        # The rectangular workspace is convex: safe endpoints imply safe walls
        # along the whole straight segment. Tolerance conservatively handles tangency.
        for point in (start, end):
            if (min(point.x, self.width - point.x,
                    point.y, self.height - point.y) <= envelope + 1e-9):
                return True
        return any(segment_distance(o.center, start, end)
                   <= o.radius + envelope + 1e-9 for o in self.obstacles)

    def detections(self):
        """Return (obstacle index, physical perimeter clearance), nearest first."""
        readings = []
        for index, obstacle in enumerate(self.obstacles):
            clearance = (hypot(self.position.x - obstacle.center.x,
                               self.position.y - obstacle.center.y)
                         - self.robot_radius - obstacle.radius)
            if clearance <= self.sensor_range:
                readings.append((index, clearance))
        return sorted(readings, key=lambda reading: reading[1])

    @property
    def wall_clearance(self):
        return min(self.position.x, self.width - self.position.x,
                   self.position.y, self.height - self.position.y) - self.robot_radius

    def start(self):
        if self.status == "IDLE":
            self.status, self.message = "RUNNING", "Ready for movement commands"

    def fail(self):
        self.status, self.message = "ERROR", "Simulated failure. Reset required."

    def reset(self):
        self.position = self._initial_position
        self.status, self.message = "IDLE", "Reset complete"

    def move(self, dx, dy):
        finite(dx, dy)
        target = Point(self.position.x + dx, self.position.y + dy)
        if self.status != "RUNNING":
            self.message = "Movement requires RUNNING state. Reset after a stop or error."
            return False
        if self._blocked(self.position, target):
            self.status = "BLOCKED"
            self.message = "Unsafe path: movement rejected. Reset required."
            return False
        self.position, self.message = target, "Movement completed safely"
        return True
