# pip libs

# custom libs
from Params.params import *
from Dataset.KITTIOdometry import *
from Params.Camera import *
from Tracking.Frames.Frame import Frame
from Tracking.Frames.ImageFeature import *


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

    pose = g2o.Isometry3d()
    left = Frame(0, pose, featurel, cam, dataset.timestamps[0], np.identity(6))
    right = Frame(0, cam.compute_right_camera_pose(pose), featurer, cam, dataset.timestamps[0], np.identity(6))

    kps_left, desps_left, idx_left = left.get_unmatched_keypoints()
    kps_right, desps_right, idx_right = right.get_unmatched_keypoints()

    matches = featurel.row_match(kps_left, desps_left, kps_right, desps_right)

    px_left = np.array([kps_left[m.queryIdx].pt for m in matches])
    px_right = np.array([kps_right[m.trainIdx].pt for m in matches])


    points = cv2.triangulatePoints(
        left.projection_matrix,
        right.projection_matrix,
        px_left.T,
        px_right.T
    ).T

    print(left.projection_matrix)
    print(right.projection_matrix)
    print(px_left.T.shape)

    pc = points[:, :3] / points[:, 3:]
    print(pc)




if __name__ == '__main__':
    params = ParamsKITTI()

    dataset = KITTIOdometry('~/Downloads/KITTI/sequences/00')
    main(dataset, params)








































