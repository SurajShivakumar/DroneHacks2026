import unittest

from dronebench.experiments import Preferences
from dronebench.mission import Mission, MissionError, Observation


def event(name, stamp, **kwargs):
    return Observation(name, stamp, f"sensor-record:{stamp}", **kwargs)


class MissionTests(unittest.TestCase):
    def advance(self, mission, names, start=1):
        for stamp, name in enumerate(names, start):
            mission.observe(event(name, stamp, full_clearance=True), now=stamp)

    def test_gate_route_needs_landing_and_disarm(self):
        mission = Mission()
        self.advance(mission, ["ready", "airborne", "corridor", "maze_exit", "tunnel", "gate_b"])
        self.assertEqual(mission.state, "LANDING")
        self.assertFalse(mission.outbound_complete)
        mission.observe(event("landed_on_pad", 7, full_clearance=True), now=7)
        self.assertEqual(mission.state, "DISARM")
        mission.observe(event("disarmed", 8), now=8)
        self.assertEqual(mission.state, "COMPLETE")
        self.assertTrue(mission.outbound_complete)
        with self.assertRaises(MissionError):
            mission.observe(event("ready", 9), now=9)

    def test_window_and_explicit_return(self):
        mission = Mission(Preferences(chicane="window", return_enabled=True))
        self.advance(mission, ["ready", "airborne", "corridor", "maze_exit", "tunnel"])
        with self.assertRaises(MissionError):
            mission.observe(event("gate_b", 6, full_clearance=True), now=6)
        self.advance(mission, ["window", "landed_on_pad", "disarmed"], 6)
        self.assertEqual(mission.state, "RETURN_PRECHECK")
        self.assertTrue(mission.outbound_complete)
        self.advance(mission, ["return_ready", "return_airborne", "return_barrier",
                               "return_landed_on_pad", "return_disarmed"], 9)
        self.assertEqual(mission.state, "COMPLETE")
        self.assertEqual(len(mission.accepted), 13)

    def test_bad_observations_cannot_advance(self):
        for observation in [event("airborne", 1), event("ready", 0),
                            event("ready", 2), event("ready", 1, ambiguous=True),
                            event("ready", 1, confidence=0.8)]:
            mission = Mission()
            with self.assertRaises(MissionError):
                mission.observe(observation, now=1)
            self.assertEqual(mission.accepted, ())
            self.assertEqual(mission.state, "PRECHECK")

    def test_point_crossing_does_not_clear_checkpoint(self):
        mission = Mission()
        self.advance(mission, ["ready", "airborne"])
        with self.assertRaises(MissionError):
            mission.observe(event("corridor", 3), now=3)
        self.assertEqual(mission.state, "FIRST_CORRIDOR")

    def test_replay_and_clock_reset_rejected(self):
        mission = Mission()
        mission.observe(event("ready", 1), now=1)
        with self.assertRaises(MissionError):
            mission.observe(event("airborne", 1), now=1.1)
        with self.assertRaises(MissionError):
            mission.tick(0)
        self.assertEqual(mission.state, "TAKEOFF")

    def test_timeout_without_events_and_return_failure_preserve_outbound(self):
        mission = Mission(stage_timeout=2)
        mission.tick(1)
        self.assertEqual(mission.tick(4), "FAULT")
        self.assertEqual(mission.fault, "stage timeout")
        returning = Mission(Preferences(return_enabled=True))
        self.advance(returning, ["ready", "airborne", "corridor", "maze_exit", "tunnel",
                                 "gate_b", "landed_on_pad", "disarmed"])
        returning.abort("return estimate lost")
        self.assertTrue(returning.outbound_complete)
        self.assertEqual(returning.state, "FAULT")

    def test_invalid_evidence_rejected(self):
        with self.assertRaises(ValueError):
            Observation("ready", 1, "")
        with self.assertRaises(ValueError):
            event("ready", float("nan"))


if __name__ == "__main__":
    unittest.main()
