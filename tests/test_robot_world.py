import math
import unittest

from robot_world import World, Rect, segment_hit


class WorldTests(unittest.TestCase):
    # Tests use World directly: no Pygame window is needed. Each test creates
    # its own world, performs an action, and checks the expected result.
    def test_manual_motion_and_diagonal_speed(self):
        for direction in ((1, 0), (1, 1)):
            world = World(obstacles=[])
            start = world.position
            world.update(0.1, direction)
            self.assertAlmostEqual(math.dist(start, world.position), 16)
            self.assertAlmostEqual(world.speed, 160)
            world.update(0.1)
            self.assertEqual(world.speed, 0)

    def test_tunneling_rejected(self):
        world = World(obstacles=[Rect(200, 100, 10, 400)])
        self.assertFalse(world.move_to((800, 300)))
        self.assertEqual(world.position, (60, 300))

    def test_radius_and_tangency(self):
        world = World(obstacles=[Rect(200, 100, 20, 300)])
        self.assertFalse(world.move_to((186, 300)))
        self.assertTrue(world.move_to((185.9, 300)))

    def test_all_boundaries(self):
        world = World(obstacles=[])
        for destination in ((14, 300), (886, 300), (60, 14), (60, 586),
                            (-10, 300), (910, 300), (60, -10), (60, 610)):
            with self.subTest(destination=destination):
                self.assertFalse(world.move_to(destination))
        self.assertEqual(world.position, (60, 300))

    def test_sensor_obstacle_wall_and_maximum(self):
        world = World(obstacles=[Rect(200, 100, 20, 400)])
        self.assertAlmostEqual(world.sensor(), 140)
        world.heading = math.pi
        self.assertAlmostEqual(world.sensor(), 60)
        world.heading = -math.pi / 2
        self.assertAlmostEqual(world.sensor(), 300)
        self.assertEqual(world.sensor(50), 50)

    def test_sensor_diagonal(self):
        world = World(obstacles=[Rect(160, 400, 20, 20)])
        world.heading = math.pi / 4
        self.assertAlmostEqual(world.sensor(), math.sqrt(2) * 100)

    def test_default_route_reaches_target_safely(self):
        # Simulate frames at 60 FPS and check every segment, not just arrival.
        world = World()
        world.toggle_mode()
        self.assertTrue(world.path)
        for _ in range(2000):
            previous = world.position
            world.update(1 / 60)
            self.assertTrue(world.path_clear(previous, world.position))
            self.assertLessEqual(world.speed, world.max_speed + 1e-6)
            if world.message == "Target reached":
                break
        self.assertLess(math.dist(world.position, world.target), 1e-6)
        world.update(0.1)
        self.assertEqual(world.speed, 0)

    def test_no_route_stops(self):
        world = World(obstacles=[Rect(400, 0, 50, 600)])
        world.toggle_mode()
        self.assertEqual(world.path, [])
        world.update(1)
        self.assertEqual(world.position, (60, 300))
        self.assertEqual(world.message, "No route to target")

    def test_mode_switch_and_replan(self):
        world = World()
        world.toggle_mode()
        world.update(0.2)
        world.toggle_mode()
        self.assertEqual(world.mode, "MANUAL")
        self.assertEqual(world.path, [])
        world.toggle_mode()
        self.assertTrue(world.path)
        self.assertTrue(world.path_clear(world.position, world.path[0]))

    def test_large_auto_step_checks_each_route_segment(self):
        world = World()
        world.toggle_mode()
        world.update(100)
        self.assertAlmostEqual(math.dist(world.position, world.target), 0)

    def test_diagonal_corner_cutting_is_blocked(self):
        world = World(obstacles=[Rect(100, 100, 20, 20)])
        self.assertFalse(world.path_clear((80, 100), (100, 80)))

    def test_invalid_inputs(self):
        world = World(obstacles=[])
        for value in (float('nan'), float('inf'), -1):
            with self.assertRaises(ValueError):
                world.update(value)
        self.assertFalse(world.move_to((float('nan'), 300)))
        with self.assertRaises(ValueError):
            World(start=(0, 0))
        with self.assertRaises(ValueError):
            world.sensor(0)

    def test_segment_parallel_and_zero_length(self):
        bounds = (10, 10, 20, 20)
        self.assertIsNone(segment_hit((0, 0), (30, 0), bounds))
        self.assertEqual(segment_hit((15, 15), (15, 15), bounds), 0)
        self.assertIsNone(segment_hit((0, 0), (0, 0), bounds))


if __name__ == '__main__':
    unittest.main()
