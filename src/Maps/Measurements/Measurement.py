# pip libs
from enum import Enum
import numpy as np

# custom libs
from Maps.Measurements.GraphMeasurement import GraphMeasurement

class Measurement(GraphMeasurement):

    Source = Enum('Measurements.Source', ['TRIANGULATION', 'TRACKING', 'REFIND'])
    Type = Enum('Measurements.Type', ['STEREO', 'LEFT', 'RIGHT'])

    def __init__(self, type, source, keypoints, descriptors):
        super().__init__() ## has keyframe & mappoint

        self.type = type
        self.source = source
        self.keypoints = keypoints # cv2.'s key point. (u,v) [left right]
        self.descriptors = descriptors # dscrp [left, right]
        self.view = None    # mappoint's position in current coordinates frame

        self.xy = np.array(self.keypoints[0].pt)
        if self.is_stereo():
            ## hmm. stereo's right y value is the same as left. thus, xyx
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
