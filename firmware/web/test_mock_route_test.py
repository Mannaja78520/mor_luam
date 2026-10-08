"""Offline timing-contract check for the UI's fake robot. Never contacts hardware."""
import random
import unittest
from unittest.mock import patch

import mock_robot


class RouteTestMockChecks(unittest.TestCase):
    def test_preparation_excluded_and_no_settings_write(self):
        clock = [0]
        with patch.object(mock_robot, "now_ms", lambda: clock[0]):
            random.seed(42)
            robot = mock_robot.Robot()
            robot.settings.update(planner="detour", navLoop=True, navTolM=0.02)
            robot.points = [{"x": 0.15, "y": 0.0}]
            robot.wheel = 180.0
            robot.start_test("direct", 0)
            prepared_at = None
            for _ in range(2000):
                clock[0] += 20
                robot.heartbeat_ms = clock[0]
                robot.step(0.02)
                state = robot.status()["nav"]["test"]
                if state["phase"] == "aligning":
                    self.assertEqual(state["elapsedMs"], 0)
                    self.assertIsNone(state["actualStartHeadingDeg"])
                elif prepared_at is None:
                    prepared_at = clock[0]
                if state["phase"] == "done":
                    break
            self.assertTrue(state["valid"])
            self.assertGreater(prepared_at, 2500)
            self.assertEqual(state["elapsedMs"], clock[0] - prepared_at)
            self.assertFalse(robot.status()["nav"]["loop"])
            self.assertEqual(robot.settings["planner"], "detour")
            self.assertTrue(robot.settings["navLoop"])
            self.assertLessEqual(min(state["actualStartHeadingDeg"], 360 - state["actualStartHeadingDeg"]), 3.5)

    def test_stopped_trial_never_valid(self):
        robot = mock_robot.Robot()
        robot.wheel = 180
        robot.start_test("direct", 0)
        robot.stop("test stop")
        self.assertFalse(robot.test["valid"])
        self.assertEqual(robot.test["phase"], "stopped")
        self.assertEqual(robot.test["elapsedMs"], 0)


if __name__ == "__main__":
    unittest.main()
