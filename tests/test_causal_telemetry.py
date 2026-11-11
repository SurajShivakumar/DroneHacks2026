from dronebench.causal_telemetry import CausalInbox


def test_future_delivery_waits_for_actual_clock_without_restamping():
    inbox = CausalInbox()
    inbox.put('position', 10.004, 20., {'x': 1.})
    assert inbox.drain(10.) == []
    assert inbox.drain(10.005) == [('position', 10.004, 20., {'x': 1.})]
    assert inbox.drain(12.) == []


def test_delayed_samples_retain_original_age_and_all_reset_evidence():
    inbox = CausalInbox()
    inbox.put('position', 1., 10., {'reset': 0})
    inbox.put('position', 2., 11., {'reset': 1})
    assert inbox.drain(5.) == [('position', 1., 10., {'reset': 0}),
                              ('position', 2., 11., {'reset': 1})]


def test_source_regression_latches_failure_even_when_future_queued():
    inbox = CausalInbox()
    inbox.put('status', 10., 20., {})
    inbox.put('status', 9., 21., {})
    assert 'regressed' in inbox.error
