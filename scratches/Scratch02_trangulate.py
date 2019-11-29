# pip libs

# custom libs
from Params.params import *
from Dataset.KITTIOdometry import *
from Params.Camera import *
from Frames.ImageFeature import *
from Frames.StereoFrame import StereoFrame

def main(dataset, params):
    cam = Camera(
        dataset.cam.fx, dataset.cam.fy, dataset.cam.cx, dataset.cam.cy,
        dataset.cam.width, dataset.cam.height,
        params.frustum_near, params.frustum_far,
        dataset.cam.baseline)

    img_0l = dataset.left[0]
    img_0r = dataset.right[0]

    featurel = ImageFeature(img_0l, params)
    featurer = ImageFeature(img_0r, params)
    featurel.extract()
    featurer.extract()

    i = 0
    timestamp = dataset.timestamps[i]
    sFrame = StereoFrame(i, g2o.Isometry3d(), featurel, featurer, cam, timestamp=timestamp)
    mappoints, measurements = sFrame.triangulate()

    # mappoints: que of mappoint
    # mappoint: object containing 3D vector of each point in point cloud in world coord.
    # measurements: object containing keypoints and descriptors of left and right cooresponding to mapppoint
    print(mappoints[0].position)


if __name__ == '__main__':
    params = ParamsKITTI()

    dataset = KITTIOdometry('~/Downloads/KITTI/sequences/00')
    main(dataset, params)








































