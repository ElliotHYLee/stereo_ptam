
import numpy as np

from LoopClosure.NearestNeighbors import NearestNeighbors

# a very simple implementation
class LoopDetection(object):
    def __init__(self, params):
        self.params = params
        self.nns = NearestNeighbors()

    def add_keyframe(self, keyframe):
        ## keyframe.feature = ImageFeature() left
        embedding = keyframe.feature.descriptors.mean(axis=0)
        self.nns.add_item(embedding, keyframe)

    def detect(self, keyframe):
        embedding = keyframe.feature.descriptors.mean(axis=0)
        kfs, ds = self.nns.search(embedding, k=20)

        if len(kfs) > 0 and kfs[0] == keyframe:
            kfs, ds = kfs[1:], ds[1:]
        if len(kfs) == 0:
            return None

        min_d = np.min(ds)
        for kf, d in zip(kfs, ds):
            if abs(kf.id - keyframe.id) < self.params.lc_min_inbetween_frames:
                continue
            if (np.linalg.norm(kf.position - keyframe.position) >
                self.params.lc_max_inbetween_distance):
                break
            if d > self.params.lc_embedding_distance or d > min_d * 1.5:
                break
            return kf
        return None
