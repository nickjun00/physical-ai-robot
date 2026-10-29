"""Pygame-independent robot logic. All distances are pixels, time is seconds."""
from dataclasses import dataclass
import heapq
import math

WIDTH, HEIGHT = 900, 600
ROBOT_RADIUS = 14
GRID = 20
# Grid points are 20 pixels apart. Smaller spacing gives finer routes but
# requires A* to search more points. Screen y coordinates increase downward.


@dataclass(frozen=True)
class Rect:
    """An obstacle: top-left corner (x, y), followed by its size."""
    x: float
    y: float
    width: float
    height: float

    def bounds(self, padding=0):
        # Return left, top, right, bottom; padding expands all four edges.
        return (self.x - padding, self.y - padding,
                self.x + self.width + padding, self.y + self.height + padding)


def segment_hit(start, end, bounds):
    """Return first contact as a fraction of the segment, or None.

    Clip a line against each pair of rectangle edges (the slab method).
    This tests the entire movement, so even large steps cannot tunnel.
    """
    # A fraction of 0 means the start; 1 means the end of the movement.
    # Keep the interval of fractions that lies inside BOTH x and y bounds.
    low, high = 0.0, 1.0
    for origin, destination, minimum, maximum in zip(
            start, end, bounds[:2], bounds[2:]):
        delta = destination - origin
        if abs(delta) < 1e-12:
            # Almost no motion on this axis: avoid dividing by zero.
            if not minimum <= origin <= maximum:
                return None
        else:
            first, last = sorted(((minimum - origin) / delta,
                                  (maximum - origin) / delta))
            low, high = max(low, first), min(high, last)
            # Disjoint intervals mean the segment misses the rectangle.
            if low > high:
                return None
    return low


class World:
    """Store robot state and rules; no window or keyboard is needed to test it."""
    def __init__(self, obstacles=None, start=(60.0, 300.0), target=(840.0, 300.0)):
        self.obstacles = list(obstacles) if obstacles is not None else [
            Rect(200, 120, 70, 330), Rect(400, 0, 70, 300),
            Rect(590, 250, 70, 300), Rect(730, 80, 90, 70)]
        self.position = tuple(start)
        self.target = tuple(target)
        self.heading = 0.0  # Radians: 0 faces right; pi/2 faces down.
        self.max_speed = 160.0
        self.speed = 0.0
        self.mode = "MANUAL"
        self.path = []  # Ordered waypoint coordinates for automatic movement.
        self.message = "WASD / arrows to move; Space for automatic navigation"
        if not self.path_clear(start, start) or not self.path_clear(target, target):
            raise ValueError("Start and target must have room for the robot")

    def path_clear(self, start, end):
        # Expanded rectangles conservatively protect the circular robot,
        # including a little extra clearance at rectangular corners.
        for x, y in (start, end):
            if not (math.isfinite(x) and math.isfinite(y)
                    and ROBOT_RADIUS < x < WIDTH - ROBOT_RADIUS
                    and ROBOT_RADIUS < y < HEIGHT - ROBOT_RADIUS):
                return False
        return all(segment_hit(start, end, obstacle.bounds(ROBOT_RADIUS)) is None
                   for obstacle in self.obstacles)

    def move_to(self, destination):
        # Commit the new position only after the entire segment is safe.
        if not self.path_clear(self.position, destination):
            return False
        self.position = tuple(destination)
        return True

    def sensor(self, maximum=300.0):
        """Forward ray distance measured from robot centre to a wall/obstacle."""
        if not math.isfinite(maximum) or maximum <= 0:
            raise ValueError("Sensor range must be positive and finite")
        x, y = self.position
        # Cosine and sine turn the facing angle into a unit direction vector.
        dx, dy = math.cos(self.heading), math.sin(self.heading)
        distance = maximum
        # The robot starts inside the window: use the forward wall intersection.
        for origin, direction, limit in ((x, dx, WIDTH), (y, dy, HEIGHT)):
            if abs(direction) > 1e-12:
                distance = min(distance, ((limit if direction > 0 else 0) - origin) / direction)
        end = (x + dx * maximum, y + dy * maximum)
        for obstacle in self.obstacles:
            hit = segment_hit(self.position, end, obstacle.bounds())
            if hit is not None:
                # Convert a segment fraction to pixels; keep the nearest hit.
                distance = min(distance, hit * maximum)
        return distance

    def plan_path(self):
        """A* on a small 8-neighbour grid with collision-checked connections."""
        start, goal = self.position, self.target
        if self.path_clear(start, goal):
            # No detour is needed when the target is directly reachable.
            return [goal]
        nodes = {(x, y) for x in range(GRID, WIDTH, GRID)
                 for y in range(GRID, HEIGHT, GRID) if self.path_clear((x, y), (x, y))}
        # Connect arbitrary robot positions to nearby grid points. Multiple
        # connections avoid choosing a single point on the wrong side of a wall.
        def nearby(point):
            return [node for node in sorted(nodes)
                    if math.dist(node, point) <= GRID * 2
                    and self.path_clear(point, node)]
        starts, finishes = nearby(start), set(nearby(goal))
        # costs: shortest distance found so far from the robot to each node.
        # parents: previous node, used later to reconstruct the route.
        # queue: a heap that returns the smallest estimated total cost first.
        # A* priority = distance so far + straight-line distance to the goal.
        costs, parents, queue = {}, {}, []
        for node in starts:
            costs[node] = math.dist(start, node)
            parents[node] = None
            heapq.heappush(queue, (costs[node] + math.dist(node, goal), costs[node], node))
        while queue:
            _, cost, node = heapq.heappop(queue)
            if cost != costs[node]:
                # A better route to this point was added after this queue entry.
                continue
            if node in finishes:
                # Follow parents backward, then reverse to get travel order.
                route = [goal, node]
                while parents[node] is not None:
                    node = parents[node]
                    route.append(node)
                return list(reversed(route))
            for dx in (-GRID, 0, GRID):
                # These offsets cover horizontal, vertical, and diagonal steps.
                for dy in (-GRID, 0, GRID):
                    neighbour = (node[0] + dx, node[1] + dy)
                    if neighbour == node or neighbour not in nodes:
                        continue
                    if not self.path_clear(node, neighbour):
                        continue
                    new_cost = cost + math.dist(node, neighbour)
                    if new_cost < costs.get(neighbour, math.inf):
                        costs[neighbour], parents[neighbour] = new_cost, node
                        heapq.heappush(queue, (new_cost + math.dist(neighbour, goal),
                                               new_cost, neighbour))
        return []

    def toggle_mode(self):
        # Re-entering automatic mode plans from the robot's current position.
        self.mode = "AUTO" if self.mode == "MANUAL" else "MANUAL"
        self.speed = 0.0
        self.path = self.plan_path() if self.mode == "AUTO" else []
        self.message = ("Following A* route" if self.path else "No route to target") if self.mode == "AUTO" else "Manual control"

    def update(self, dt, direction=(0, 0)):
        """Advance one frame; dt is elapsed seconds, direction is keyboard input."""
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Time step must be finite and nonnegative")
        if not all(math.isfinite(v) for v in direction):
            raise ValueError("Direction must be finite")
        self.speed = 0.0
        if dt == 0:
            return
        before = self.position
        if self.mode == "AUTO":
            # Distance = speed × time. Spend this frame's distance across
            # waypoints without overshooting any turn in the route.
            budget = self.max_speed * dt
            travelled = 0.0
            while self.path and budget > 0:
                waypoint = self.path[0]
                distance = math.dist(self.position, waypoint)
                if distance < 1e-8:
                    self.path.pop(0)
                    continue
                dx, dy = waypoint[0] - self.position[0], waypoint[1] - self.position[1]
                self.heading = math.atan2(dy, dx)
                step = min(budget, distance)
                # Dividing by distance makes a unit vector toward the waypoint.
                destination = (self.position[0] + dx / distance * step,
                               self.position[1] + dy / distance * step)
                if not self.move_to(destination):
                    self.path = []
                    self.message = "Route blocked; toggle mode to replan"
                    break
                travelled += step
                budget -= step
                if step == distance:
                    self.path.pop(0)
            self.speed = travelled / dt
            if math.dist(self.position, self.target) < 1e-6:
                self.message = "Target reached"
        else:
            dx, dy = direction
            length = math.hypot(dx, dy)
            if length:
                # Normalize keyboard input so diagonal motion is not faster.
                self.heading = math.atan2(dy, dx)
                distance = self.max_speed * dt
                self.move_to((before[0] + dx / length * distance,
                              before[1] + dy / length * distance))
            # Report actual speed: a rejected movement has speed zero.
            self.speed = math.dist(before, self.position) / dt
