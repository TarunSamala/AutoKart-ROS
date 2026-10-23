"""Bounded observations: arrival rates and source timestamps are different facts."""
from collections import deque
import math

def safe_value(value):
    if isinstance(value, dict):
        return {str(k): safe_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, bytes, bytearray)):
        return [safe_value(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value

class TopicEvidence:
    def __init__(self):
        self.arrivals = deque(maxlen=128)
        self.stamp_ns = None
        self.nonmonotonic_stamps = 0
        self.payload = {}
        self.malformed = False

    def observe(self, arrival_s, stamp_ns=None, payload=None, malformed=False):
        self.arrivals.append(arrival_s)
        if stamp_ns is not None:
            if self.stamp_ns is not None and stamp_ns <= self.stamp_ns:
                self.nonmonotonic_stamps += 1
            self.stamp_ns = stamp_ns
        self.payload = safe_value(payload) if isinstance(payload, dict) else {}
        self.malformed = malformed

    def snapshot(self, now_ns):
        span = self.arrivals[-1] - self.arrivals[0] if len(self.arrivals) > 1 else 0
        rate = (len(self.arrivals)-1)/span if span > 0 else None
        age = (now_ns-self.stamp_ns)/1e9 if self.stamp_ns is not None else None
        return {"observed_arrival_rate_hz": rate, "source_age_s": age,
                "nonmonotonic_stamp_count": self.nonmonotonic_stamps,
                "clock_domain_verified": False,
                "malformed": self.malformed, "reported": self.payload,
                "queue_backlog": None, "usb_resets": None}
