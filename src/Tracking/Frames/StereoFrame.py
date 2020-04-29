# pip libs
import cv2
import numpy as np
from queue import Queue
from threading import Thread

# custom libs
from Tracking.Frames.Frame import Frame
from Maps.Measurements.Measurement import Measurement
from Maps.MapPoints.MapPoint import MapPoint

class StereoFrame(Frame):
    def __init__(self, idx, pose, feature, right_feature, cam, right_cam=None, timestamp=None, pose_covariance=np.identity(6)):
        super().__init__(idx, pose, feature, cam, timestamp, pose_covariance)
        self.left  = Frame(idx, pose, feature, cam, timestamp, pose_covariance)
        self.right = Frame(idx, cam.compute_right_camera_pose(pose), right_feature, right_cam or cam, timestamp, pose_covariance)

    ## me
    ## returns list of pairs, pair = (i, measObj),
    ## i = matched idx,
    ## measObj = (feature[i], descriptor[i])

    def find_matches(self, source, points, descriptors):
        q2 = Queue()
        def find_right(points, descriptors, q):
            m = dict(self.right.find_matches(points, descriptors))
            q.put(m)
        t2 = Thread(target=find_right, args=(points, descriptors, q2))
        t2.start()
        matches_left = dict(self.left.find_matches(points, descriptors))
        t2.join()
        matches_right = q2.get()

        measurements = []
        for i, j in matches_left.items():
            if i in matches_right:
                j2 = matches_right[i]

                y1 = self.left.get_keypoint(j).pt[1]
                y2 = self.right.get_keypoint(j2).pt[1]
                if abs(y1 - y2) > 2.5:    # epipolar constraint
                    continue   # TODO: choose one

                meas = Measurement(Measurement.Type.STEREO,
                                   source,
                                   [self.left.get_keypoint(j), self.right.get_keypoint(j2)],
                                   [self.left.get_descriptor(j), self.right.get_descriptor(j2)])
                measurements.append((i, meas))
                self.left.set_matched(j)
                self.right.set_matched(j2)
            else:
                meas = Measurement(Measurement.Type.LEFT,
                                    source,
                                    [self.left.get_keypoint(j)],
                                    [self.left.get_descriptor(j)])
                measurements.append((i, meas))
                self.left.set_matched(j)

        for i, j in matches_right.items():
            if i not in matches_left:
                meas = Measurement(Measurement.Type.RIGHT,
                                   source,
                                   [self.right.get_keypoint(j)],
                                   [self.right.get_descriptor(j)])
                measurements.append((i, meas))
                self.right.set_matched(j)

        return measurements

    def match_mappoints(self, mappoints, source):
        points = []
        descriptors = []
        for mappoint in mappoints:
            points.append(mappoint.position)
            descriptors.append(mappoint.descriptor)
        matched_measurements = self.find_matches(source, points, descriptors)

        measurements = []
        for i, meas in matched_measurements:
            meas.mappoint = mappoints[i]
            measurements.append(meas)
        return measurements

    def triangulate(self):
        kps_left, desps_left, idx_left = self.left.get_unmatched_keypoints()
        kps_right, desps_right, idx_right = self.right.get_unmatched_keypoints()

        mappoints, matches = self.triangulate_points(kps_left, desps_left, kps_right, desps_right)

        measurements = []
        for mappoint, (i, j) in zip(mappoints, matches):
            meas = Measurement(Measurement.Type.STEREO,
                               Measurement.Source.TRIANGULATION,
                               [kps_left[i], kps_right[j]],
                               [desps_left[i], desps_right[j]])
            meas.mappoint = mappoint
            meas.view = self.transform(mappoint.position)
            measurements.append(meas)

            self.left.set_matched(idx_left[i])
            self.right.set_matched(idx_right[j])

        return mappoints, measurements

    def triangulate_points(self, kps_left, desps_left, kps_right, desps_right):
        matches = self.feature.row_match(kps_left, desps_left, kps_right, desps_right)
        assert len(matches) > 0

        px_left = np.array([kps_left[m.queryIdx].pt for m in matches])
        px_right = np.array([kps_right[m.trainIdx].pt for m in matches])

        # print(self.left.projection_matrix)
        # points in global coord
        points = cv2.triangulatePoints(self.left.projection_matrix, # 3D world_blogal to 2D img in body frame
                                       self.right.projection_matrix,
                                       px_left.transpose(),
                                       px_right.transpose()).transpose()  # shape: (N, 4)

        points = points[:, :3] / points[:, 3:]

        can_view = np.logical_and(self.left.can_view(points), self.right.can_view(points))

        mappoints = []
        matchList = []
        for i, point in enumerate(points):
            if not can_view[i]:
                continue
            normal = point - self.position
            normal = normal / np.linalg.norm(normal)

            color = self.left.get_color(px_left[i])

            mappoint = MapPoint(point, normal, desps_left[matches[i].queryIdx], color)
            mappoints.append(mappoint)
            matchList.append((matches[i].queryIdx, matches[i].trainIdx))

        return mappoints, matchList

    def update_pose(self, pose):
        super().update_pose(pose)
        self.right.update_pose(pose)
        self.left.update_pose(self.cam.compute_right_camera_pose(pose))

    def can_view(self, mappoints):
        points = []
        point_normals = []

        for p in mappoints:
            points.append(p.position)
            point_normals.append(p.normal)
        points = np.asarray(points)
        point_normals = np.asarray(point_normals) ## N by 3

        normals = points - self.position
        normals /= np.linalg.norm(normals, axis=-1, keepdims=True)

        dotOfNormals = np.sum(point_normals * normals, axis=1)
        cos = np.clip(dotOfNormals, -1, 1)
        parallel = np.arccos(cos) < (np.pi / 4)
        ## parallel: within 45 deg.

        can_view = np.logical_or(self.left.can_view(points), self.right.can_view(points))

        ## me
        ## why AND-gate parallel? -> considering blocking?
        ## Nov.29.2019
        return np.logical_and(parallel, can_view)

    def to_keyframe(self):
        from Tracking.Frames.KeyFrame import KeyFrame
        return KeyFrame(
            self.idx, self.pose,
            self.left.feature, self.right.feature,
            self.cam, self.right.cam,
            self.pose_covariance)
