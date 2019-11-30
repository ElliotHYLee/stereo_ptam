from threading import Lock
from collections import defaultdict, Counter
from itertools import chain

class GraphKeyFrame(object):
    def __init__(self):
        self.id = None
        self.meas = dict()
        self.covisible = defaultdict(int)
        self._lock = Lock()

    def __hash__(self):
        return self.id

    def __eq__(self, rhs):
        return (isinstance(rhs, GraphKeyFrame) and
            self.id == rhs.id)

    def __lt__(self, rhs):
        return self.id < rhs.id   # predate

    def __le__(self, rhs):
        return self.id <= rhs.id

    def measurements(self):
        with self._lock:
            return self.meas.keys()

    def mappoints(self):
        with self._lock:
            return self.meas.values()

    def add_measurement(self, m):
        with self._lock:
            self.meas[m] = m.mappoint

    def remove_measurement(self, m):
        with self._lock:
            try:
                del self.meas[m]
            except KeyError:
                pass

    def covisibility_keyframes(self):
        with self._lock:
            return self.covisible.copy()  # shallow copy

    def add_covisibility_keyframe(self, kf):
        with self._lock:
            self.covisible[kf] += 1
