# pip libs
from threading import Lock

# custom libs
from Tracking.Frames.StereoFrame import StereoFrame
from Tracking.Frames.GraphKeyFrame import GraphKeyFrame

class KeyFrame(GraphKeyFrame, StereoFrame):

    _id = 0
    _id_lock = Lock()

    def __init__(self, *args, **kwargs):
        GraphKeyFrame.__init__(self)
        StereoFrame.__init__(self, *args, **kwargs)

        with KeyFrame._id_lock:
            self.id = KeyFrame._id
            KeyFrame._id += 1

        self.reference_keyframe = None
        self.reference_constraint = None
        self.preceding_keyframe = None
        self.preceding_constraint = None
        self.loop_keyframe = None
        self.loop_constraint = None
        self.fixed = False

    def update_reference(self, reference=None):
        if reference is not None:
            self.reference_keyframe = reference
        self.reference_constraint = (self.reference_keyframe.pose.inverse() * self.pose)

    def update_preceding(self, preceding=None):
        if preceding is not None:
            self.preceding_keyframe = preceding
        self.preceding_constraint = (self.preceding_keyframe.pose.inverse() * self.pose)

    def set_loop(self, keyframe, constraint):
        self.loop_keyframe = keyframe
        self.loop_constraint = constraint

    def is_fixed(self):
        return self.fixed

    def set_fixed(self, fixed=True):
        self.fixed = fixed

