class GraphMeasurement(object):
    def __init__(self):
        self.keyframe = None
        self.mappoint = None

    @property
    def id(self):
        return (self.keyframe.id, self.mappoint.id)

    def __hash__(self):
        return hash(self.id)

    def __eq__(self, rhs):
        return (isinstance(rhs, GraphMeasurement) and self.id == rhs.id)
