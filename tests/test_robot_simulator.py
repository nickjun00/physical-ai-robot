import unittest

from robot_simulator import Point, RectangleObstacle, RobotSimulator, segment_rectangle_hit


class SegmentCollisionTests(unittest.TestCase):
    def test_detects_thin_obstacle_crossed_by_long_move(self):
        obstacle = RectangleObstacle("thin wall", 5, -1, 0.01, 2)
        self.assertAlmostEqual(segment_rectangle_hit(Point(0, 0), Point(10, 0), obstacle), 0.5)

    def test_parallel_segment_outside_rectangle_does_not_hit(self):
        obstacle = RectangleObstacle("box", 2, 2, 1, 1)
        self.assertIsNone(segment_rectangle_hit(Point(0, 0), Point(10, 0), obstacle))


class RobotSafetyTests(unittest.TestCase):
    def setUp(self):
        self.wall = RectangleObstacle("wall", 2, -1, 1, 2)

    def test_robot_stops_before_wall_using_radius_and_margin(self):
        robot = RobotSimulator([self.wall], robot_radius=0.2, safety_margin=0.1)
        result = robot.move_to(Point(5, 0))
        self.assertTrue(result.blocked)
        self.assertEqual(result.obstacle, "wall")
        self.assertLess(result.position.x, 1.7)
        self.assertGreater(result.position.x, 1.699999)
        self.assertAlmostEqual(robot.position.x, result.position.x)

    def test_first_obstacle_on_path_wins(self):
        robot = RobotSimulator(
            [RectangleObstacle("far", 5, -1, 1, 2), RectangleObstacle("near", 2, -1, 1, 2)],
            robot_radius=0,
            safety_margin=0,
        )
        result = robot.move_to(Point(10, 0))
        self.assertTrue(result.blocked)
        self.assertEqual(result.obstacle, "near")

    def test_sensor_reports_edge_distance_and_sorts_readings(self):
        robot = RobotSimulator(
            [RectangleObstacle("far", 5, 0, 1, 1), RectangleObstacle("near", 2, 0, 1, 1)],
            robot_radius=0.5,
        )
        readings = robot.detect_obstacles(10)
        self.assertEqual([reading.name for reading in readings], ["near", "far"])
        self.assertAlmostEqual(readings[0].distance, 1.5)
        self.assertEqual(robot.detect_obstacles(1.49), ())

    def test_safe_path_reaches_target(self):
        robot = RobotSimulator([self.wall])
        result = robot.move_to(Point(0, 3))
        self.assertFalse(result.blocked)
        self.assertEqual(result.position, Point(0, 3))

    def test_starting_in_unsafe_position_is_blocked_without_motion(self):
        robot = RobotSimulator([self.wall], position=Point(2.5, 0))
        result = robot.move_to(Point(5, 0))
        self.assertTrue(result.blocked)
        self.assertEqual(result.position, Point(2.5, 0))


if __name__ == "__main__":
    unittest.main()
