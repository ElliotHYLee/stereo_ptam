import numpy as np
import g2o

class Frame(object):
    def __init__(self, idx, pose, left_feature, cam, timestamp=None, pose_covariance = np.identity(6)):
        self.idx = idx
        self.pose = pose    # g2o.Isometry3d
        self.left_feature = left_feature
        self.cam = cam
        self.timestamp = timestamp
        self.image = left_feature.image

        self.orientation = pose.orientation()
        self.position = pose.position()
        self.pose_covariance = pose_covariance

        self.transform_matrix = pose.inverse().matrix()[:3] # shape: (3, 4)
        self.projection_matrix = (self.cam.intrinsic.dot(self.transform_matrix))  # from world frame to image

    # batch version
    def can_view(self, points, ground=False, margin=20):    # Frustum Culling
        points = np.transpose(points)
        (u, v), depth = self.project(self.transform(points))

        if ground:
            return np.logical_and.reduce([
                depth >= self.cam.frustum_near,
                depth <= self.cam.frustum_far,
                u >= - margin,
                u <= self.cam.width + margin])
        else:
            return np.logical_and.reduce([
                depth >= self.cam.frustum_near,
                depth <= self.cam.frustum_far,
                u >= - margin,
                u <= self.cam.width + margin,
                v >= - margin,
                v <= self.cam.height + margin])


    def update_pose(self, pose):
        if isinstance(pose, g2o.SE3Quat):
            self.pose = g2o.Isometry3d(pose.orientation(), pose.position())
        else:
            self.pose = pose
        self.orientation = self.pose.orientation()
        self.position = self.pose.position()

        self.transform_matrix = self.pose.inverse().matrix()[:3]
        self.projection_matrix = (self.cam.intrinsic.dot(self.transform_matrix))

    def transform(self, points):    # from world coordinates
        '''
        Transform points from world coordinates frame to camera frame.
        Args:
            points: a point or an array of points, of shape (3,) or (3, N).
        '''
        R = self.transform_matrix[:3, :3]
        if points.ndim == 1:
            t = self.transform_matrix[:3, 3]
        else:
            t = self.transform_matrix[:3, 3:]
        return R.dot(points) + t

    def project(self, points):
        '''
        Project points from camera frame to image's pixel coordinates.
        Args:
            points: a point or an array of points, of shape (3,) or (3, N).
        Returns:
            Projected pixel coordinates, and respective depth.
        '''
        projection = self.cam.intrinsic.dot(points / points[-1:])
        return projection[:2], points[-1]

    def find_matches(self, points, descriptors):
        '''
        Match to points from world frame.
        Args:
            points: a list/array of points. shape: (N, 3)
            descriptors: a list of feature descriptors. length: N
        Returns:
            List of successfully matched (queryIdx, trainIdx) pairs.
        '''
        points = np.transpose(points)
        proj, _ = self.project(self.transform(points))
        proj = proj.transpose()
        return self.left_feature.find_matches(proj, descriptors)

    def get_keypoint(self, i):
        return self.left_feature.get_keypoint(i)
    def get_descriptor(self, i):
        return self.left_feature.get_descriptor(i)
    def get_color(self, pt):
        return self.left_feature.get_color(pt)
    def set_matched(self, i):
        self.left_feature.set_matched(i)
    def get_unmatched_keypoints(self):
        return self.left_feature.get_unmatched_keypoints()
