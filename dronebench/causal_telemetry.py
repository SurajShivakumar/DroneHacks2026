"""Select existing source samples causally; never alter timestamps or state."""
from collections import deque
from math import isfinite


class CausalInbox:
    """ROS /clock and DDS samples have independent callback delivery order.

    Hold a slightly future source sample until that clock actually arrives.
    Frozen, regressed or delayed inputs never become fresh by being queued.
    Consumers still apply their normal source-age and arrival-age budgets.
    """
    def __init__(self, capacity=256):
        self.queues = {}
        self.last_received = {}
        self.error = None
        self.capacity = capacity

    def put(self, name, stamp_s, received_wall, data):
        if not isfinite(stamp_s) or stamp_s < 0 or not isfinite(received_wall):
            self.error = 'invalid telemetry timestamp'
            return
        previous = self.last_received.get(name)
        if previous is not None and (stamp_s < previous[0] or received_wall < previous[1]):
            self.error = name+': source/arrival clock regressed'
            return
        self.last_received[name] = stamp_s, received_wall
        queue = self.queues.setdefault(name, deque(maxlen=self.capacity))
        if len(queue) == self.capacity:
            self.error = name+': causal telemetry queue overflow'
            return
        queue.append((stamp_s, received_wall, dict(data)))

    def drain(self, now_sim):
        if not isfinite(now_sim) or now_sim < 0:
            self.error = 'invalid simulation clock'
            return []
        ready = []
        for name, queue in self.queues.items():
            # Consume in source order, retaining actual arrival times. Deliver
            # each progression so reset counters cannot be hidden by a shortcut.
            while queue and queue[0][0] <= now_sim:
                stamp, wall, data = queue.popleft()
                ready.append((name, stamp, wall, data))
        return ready
