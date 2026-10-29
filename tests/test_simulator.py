import unittest

from simulator import Obstacle, Point, Simulator, segment_distance


class SimulatorTests(unittest.TestCase):
    def running(self, **kwargs):
        sim = Simulator(**kwargs)
        sim.start()
        return sim

    def test_safe_move_and_zero_move(self):
        sim = self.running()
        self.assertTrue(sim.move(2, 1))
        self.assertEqual(sim.position, Point(3, 2))
        self.assertTrue(sim.move(0, 0))

    def test_long_move_cannot_tunnel_beyond_sensor_range(self):
        sim = self.running(obstacles=[Obstacle(Point(5, 1), 0.5)], sensor_range=1)
        self.assertEqual(sim.detections(), [])
        self.assertFalse(sim.move(8, 0))
        self.assertEqual(sim.position, Point(1, 1))
        self.assertEqual(sim.status, "BLOCKED")

    def test_diagonal_crossing(self):
        sim = self.running(obstacles=[Obstacle(Point(5, 5), 0.5)])
        self.assertFalse(sim.move(8, 8))

    def test_endpoint_collision(self):
        sim = self.running(obstacles=[Obstacle(Point(3, 1), 0.5)])
        self.assertFalse(sim.move(2, 0))

    def test_tangent_footprint_and_margin(self):
        for y in (1.5, 1.75, 1.85):
            with self.subTest(y=y):
                sim = self.running(obstacles=[Obstacle(Point(5, y), 0.5)])
                self.assertFalse(sim.move(8, 0))

    def test_near_miss_is_allowed(self):
        sim = self.running(obstacles=[Obstacle(Point(5, 1.851), 0.5)])
        self.assertTrue(sim.move(8, 0))

    def test_obstacle_behind_or_beyond_segment(self):
        sim = self.running(position=Point(3, 1), obstacles=[
            Obstacle(Point(1, 1), 0.5), Obstacle(Point(8, 1), 0.5)])
        self.assertTrue(sim.move(2, 0))

    def test_reverse_path(self):
        sim = self.running(position=Point(9, 1), obstacles=[Obstacle(Point(5, 1), 0.5)])
        self.assertFalse(sim.move(-8, 0))

    def test_all_walls_include_footprint_and_margin(self):
        for target in (Point(0.35, 5), Point(9.65, 5), Point(5, 0.35), Point(5, 9.65),
                       Point(-1, 5), Point(11, 5), Point(5, -1), Point(5, 11)):
            with self.subTest(target=target):
                sim = self.running(position=Point(5, 5))
                self.assertFalse(sim.move(target.x - 5, target.y - 5))
                self.assertEqual(sim.position, Point(5, 5))

    def test_detections_measure_perimeter_sort_and_include_range_boundary(self):
        sim = Simulator(sensor_range=2, obstacles=[
            Obstacle(Point(3.75, 1), 0.5), Obstacle(Point(2.5, 1), 0.5),
            Obstacle(Point(3.751, 1), 0.5)])
        self.assertEqual(sim.detections(), [(1, 0.75), (0, 2.0)])
        self.assertAlmostEqual(sim.wall_clearance, 0.75)

    def test_readings_update_after_move(self):
        sim = self.running(obstacles=[Obstacle(Point(5, 1), 0.5)])
        self.assertEqual(sim.detections(), [])
        self.assertTrue(sim.move(2, 0))
        self.assertEqual(sim.detections(), [(0, 1.25)])

    def test_idle_error_and_blocked_states_prevent_motion(self):
        sim = Simulator()
        self.assertFalse(sim.move(1, 0))
        sim.start()
        sim.fail()
        sim.start()
        self.assertEqual(sim.status, "ERROR")
        self.assertFalse(sim.move(1, 0))
        sim.reset()
        sim.start()
        self.assertFalse(sim.move(-1, 0))
        sim.start()
        self.assertEqual(sim.status, "BLOCKED")
        self.assertFalse(sim.move(1, 0))

    def test_reset_restores_position_and_requires_start(self):
        sim = self.running()
        sim.move(2, 0)
        sim.fail()
        sim.reset()
        self.assertEqual(sim.position, Point(1, 1))
        self.assertEqual(sim.status, "IDLE")
        self.assertFalse(sim.move(1, 0))
        sim.start()
        self.assertTrue(sim.move(1, 0))

    def test_invalid_configuration(self):
        for kwargs in ({"width": 0}, {"height": -1}, {"robot_radius": 0},
                       {"safety_margin": -1}, {"sensor_range": 0},
                       {"width": float("inf")}, {"position": Point(0, 0)},
                       {"obstacles": [Obstacle(Point(1, 1), 1)]}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Simulator(**kwargs)
        with self.assertRaises(ValueError):
            Obstacle(Point(2, 2), -1)

    def test_nonfinite_commands_never_mutate_state(self):
        sim = self.running()
        for value in (float("nan"), float("inf"), -float("inf")):
            for dx, dy in ((value, 0), (0, value)):
                with self.subTest(dx=dx, dy=dy), self.assertRaises(ValueError):
                    sim.move(dx, dy)
                self.assertEqual(sim.position, Point(1, 1))
                self.assertEqual(sim.status, "RUNNING")

    def test_segment_distance_degenerate_and_endpoints(self):
        self.assertEqual(segment_distance(Point(4, 5), Point(1, 1), Point(1, 1)), 5)
        self.assertEqual(segment_distance(Point(0, 1), Point(1, 1), Point(2, 1)), 1)
        self.assertEqual(segment_distance(Point(3, 1), Point(1, 1), Point(2, 1)), 1)


if __name__ == "__main__":
    unittest.main()
