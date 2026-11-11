from dataclasses import replace
import math
from pathlib import Path
import tempfile
import unittest

from dronebench.experiments import Preferences, create_manifest
from dronebench.speed import Limits, Sample, ideal_duration, speed_profile


class SpeedTests(unittest.TestCase):
    def setUp(self):
        self.limits = Limits(cruise=3, acceleration=1, braking=1,
                             lateral=1, latency=0.2, margin=0.3)
        self.samples = [Sample(i * 0.1, 0, 20) for i in range(81)]

    def test_straight_accelerates_then_brakes(self):
        speeds = speed_profile(self.samples, self.limits)
        self.assertEqual(speeds[0], 0)
        self.assertEqual(speeds[-1], 0)
        self.assertGreater(speeds[40], speeds[10])
        for a, b, va, vb in zip(self.samples, self.samples[1:], speeds, speeds[1:]):
            self.assertLessEqual(vb * vb - va * va, 2 * (b.distance - a.distance) + 1e-9)
            self.assertLessEqual(va * va - vb * vb, 2 * (b.distance - a.distance) + 1e-9)

    def test_brakes_before_corner(self):
        straight = speed_profile(self.samples, self.limits)
        path = list(self.samples)
        path[40] = replace(path[40], curvature=4)
        speeds = speed_profile(path, self.limits)
        self.assertLessEqual(speeds[40], 0.5)
        self.assertLess(speeds[35], straight[35])

    def test_visibility_accounts_for_latency_and_margin(self):
        path = [replace(s, free_distance=0.7) for s in self.samples]
        speeds = speed_profile(path, self.limits)
        for speed in speeds:
            self.assertLessEqual(speed * 0.2 + speed**2 / 2, 0.4 + 1e-9)

    def test_unknown_space_stops_profile(self):
        path = list(self.samples)
        path[40] = replace(path[40], free_distance=0)
        self.assertEqual(speed_profile(path, self.limits)[40], 0)

    def test_unachievable_initial_braking_rejected(self):
        path = [Sample(0, 0, 20), Sample(0.1, 0, 20)]
        with self.assertRaises(ValueError):
            speed_profile(path, self.limits, start_speed=2)

    def test_invalid_values_rejected(self):
        for value in (math.nan, math.inf, -1):
            with self.assertRaises(ValueError):
                Limits(cruise=value)
        with self.assertRaises(ValueError):
            Sample(0, 0, 1, confidence=2)
        with self.assertRaises(ValueError):
            speed_profile([self.samples[0], self.samples[0]])

    def test_ideal_time_analytical_case(self):
        self.assertEqual(ideal_duration([Sample(0, 0, 4), Sample(2, 0, 4)], [1, 1]), 2)
        with self.assertRaises(ValueError):
            ideal_duration([Sample(0, 0, 4), Sample(2, 0, 4)], [0, 0])


class ExperimentTests(unittest.TestCase):
    def test_preferences_cannot_raise_qualified_speed(self):
        with self.assertRaises(ValueError):
            Preferences(speed_scale=1.5)
        with self.assertRaises(ValueError):
            Preferences(chicane="skip_tunnel")

    def test_manifest_never_claims_flight(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.json"
            path.write_text("{}", encoding="utf-8")
            result = create_manifest("openfly", Preferences(), [path])
        self.assertEqual(result["status"], "prepared_not_flown")
        self.assertIsNone(result["recording"])
        self.assertFalse(result["flight_approved"])
        self.assertEqual(len(result["inputs"][0]["sha256"]), 64)


if __name__ == "__main__":
    unittest.main()
