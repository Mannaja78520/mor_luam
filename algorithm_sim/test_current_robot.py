"""CCW frame and current-robot examples. No hardware or GUI is contacted."""
import math
import random
import unittest
from detour_steer import unit, bearing, steer_error, plan_leg, measured_robot, comparison_route, ideal_route, detour_legs


class CurrentRobotChecks(unittest.TestCase):
    def test_ccw_frame_and_right_hand_goal(self):
        self.assertAlmostEqual(unit(math.pi / 2)[1], 1)
        self.assertAlmostEqual(math.degrees(bearing(0, 1)), 90)
        self.assertAlmostEqual(math.degrees(steer_error(math.radians(354), 0)), 354)
        r = measured_robot()
        self.assertEqual(plan_leg(math.radians(354), .3, r).kind, 'detour')
        self.assertEqual(plan_leg(math.radians(354), 2, r).kind, 'direct')
    def test_two_metre_total_matches_browser_model(self):
        points = comparison_route()
        length = 0; previous = (0, 0)
        for point in points:
            leg = math.dist(previous, point); self.assertAlmostEqual(leg, 2 / 7)
            length += leg; previous = point
        self.assertAlmostEqual(length, 2)
        direct = ideal_route(points, use_detour=False)
        detour = ideal_route(points, use_detour=True)
        self.assertAlmostEqual(direct['time_s'], 137.81666666666666)
        self.assertAlmostEqual(detour['time_s'], 110.51600541905924)
        self.assertEqual(detour['detours'], 4)
    def test_geometry_reaches_goal_in_ccw_frame(self):
        rng = random.Random(2026); checked = 0
        for _ in range(3000):
            phi, d = rng.uniform(math.pi, 2 * math.pi), rng.uniform(.05, .8)
            p = plan_leg(phi, d, measured_robot())
            if p.kind != 'detour': continue
            a, b = detour_legs(phi, d, p.beta)
            self.assertAlmostEqual(a + b * math.cos(p.beta), d * math.cos(phi))
            self.assertAlmostEqual(b * math.sin(p.beta), d * math.sin(phi))
            checked += 1
        self.assertGreater(checked, 50)


if __name__ == '__main__': unittest.main()
