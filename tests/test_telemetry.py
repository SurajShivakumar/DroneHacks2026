import unittest
import json
import numpy as np

from dronebench.telemetry import StreamStats, json_safe


class StreamTimingTests(unittest.TestCase):
    def test_actual_ros_numeric_array_scalars_serialize_without_losing_integer_precision(self):
        value={'position':list(np.array([.25,float('nan')],dtype=np.float32)),
               'timestamp':np.uint64(2**60+3),'valid':True}
        encoded=json.loads(json.dumps(json_safe(value),allow_nan=False))
        self.assertEqual(encoded,{'position':[.25,'nan'],'timestamp':2**60+3,'valid':True})
        self.assertIs(encoded['valid'],True)
    def test_unknown_covariance_stays_explicit_in_json(self):
        self.assertEqual(json_safe({"variance": [float("nan"), 1.0]}),
                         {"variance": ["nan", 1.0]})

    def test_frozen_source_is_not_mistaken_for_fresh_data(self):
        stream = StreamStats()
        for receive in (10, 11, 12):
            stream.observe(receive, 4)
        report = stream.summary(12)
        self.assertEqual(report["last_receive_age_wall_s"], 0)
        self.assertFalse(report["source_clock_observed_advancing"])

    def test_clock_reset_and_stale_callback_are_reported(self):
        stream = StreamStats()
        stream.observe(10, 8)
        stream.observe(11, 9)
        stream.observe(12, 0)
        report = stream.summary(15)
        self.assertEqual(report["source_regressions"], 1)
        self.assertEqual(report["last_receive_age_wall_s"], 3)

    def test_missing_stream_cannot_appear_healthy(self):
        report = StreamStats().summary(10)
        self.assertEqual(report["messages"], 0)
        self.assertIsNone(report["last_receive_age_wall_s"])
        self.assertFalse(report["source_clock_observed_advancing"])
