import numpy as np
import os
from collections import defaultdict, namedtuple
from Dataset.ImageReader import ImageReader

class KITTIOdometry(object):   # without lidar
    '''
    path example: 'path/to/your/KITTI odometry dataset/sequences/00'
    '''
    def __init__(self, path):
        Cam = namedtuple('cam', 'fx fy cx cy width height baseline')
        cam00_02 = Cam(718.856, 718.856, 607.1928, 185.2157, 1241, 376, 0.5371657)
        cam03 = Cam(721.5377, 721.5377, 609.5593, 172.854, 1241, 376, 0.53715)
        cam04_12 = Cam(707.0912, 707.0912, 601.8873, 183.1104, 1241, 376, 0.53715)

        path = os.path.expanduser(path)
        timestamps = np.loadtxt(os.path.join(path, 'times.txt'))
        self.left = ImageReader(self.listdir(os.path.join(path, 'image_2')), timestamps)
        self.right = ImageReader(self.listdir(os.path.join(path, 'image_3')),timestamps)

        assert len(self.left) == len(self.right)
        self.timestamps = self.left.timestamps

        sequence = int(path.strip(os.path.sep).split(os.path.sep)[-1])
        if sequence < 3:
            self.cam = cam00_02
        elif sequence == 3:
            self.cam = cam03
        elif sequence < 13:
            self.cam = cam04_12

    def sort(self, xs):
        return sorted(xs, key=lambda x:float(x[:-4]))

    def listdir(self, dir):
        files = [_ for _ in os.listdir(dir) if _.endswith('.png')]
        return [os.path.join(dir, _) for _ in self.sort(files)]

    def __len__(self):
        return len(self.left)
