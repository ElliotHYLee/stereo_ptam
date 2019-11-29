# pip libs
from enum import Enum
import numpy as np

# custom libs
from Maps.Measurement.GraphMeasurement import GraphMeasurement

class Measurement(GraphMeasurement):

    Source = Enum('Measurement.Source', ['TRIANGULATION', 'TRACKING', 'REFIND'])
    Type = Enum('Measurement.Type', ['STEREO', 'LEFT', 'RIGHT'])

    def __init__(self, type, source, keypoints, descriptors):
        super().__init__()

        self.type = type
        self.source = source
        self.keypoints = keypoints
        self.descriptors = descriptors
        self.view = None    # mappoint's position in current coordinates frame

        self.xy = np.array(self.keypoints[0].pt)
        if self.is_stereo():
            self.xyx = np.array([*keypoints[0].pt, keypoints[1].pt[0]])

        self.triangulation = (source == self.Source.TRIANGULATION)

    def get_descriptor(self, i=0):
        return self.descriptors[i]
    def get_keypoint(self, i=0):
        return self.keypoints[i]

    def get_descriptors(self):
        return self.descriptors
    def get_keypoints(self):
        return self.keypoints

    def is_stereo(self):
        return self.type == Measurement.Type.STEREO
    def is_left(self):
        return self.type == Measurement.Type.LEFT
    def is_right(self):
        return self.type == Measurement.Type.RIGHT

    def from_triangulation(self):
        return self.triangulation
    def from_tracking(self):
        return self.source == Measurement.Source.TRACKING
    def from_refind(self):
        return self.source == Measurement.Source.REFIND
