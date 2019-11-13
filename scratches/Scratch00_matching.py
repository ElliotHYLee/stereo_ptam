import g2o
from Components.Camera import Camera
import numpy as np
from Feature.ImageFeature import ImageFeature
from Dataset.KITTIOdometry import KITTIOdometry
from Params.params import ParamsKITTI
from Components.StereoFrame import StereoFrame
import cv2

dataset = KITTIOdometry('~/Downloads/KITTI/sequences/00')
cam = Camera(718.856, 718.856, 607.1928, 185.2157, 1241, 376, 0.1, 1000, 0.5371657)
params = ParamsKITTI()
i = 0
featurel = ImageFeature(dataset.left[i], params)
featurer = ImageFeature(dataset.right[i], params)

timestamp = dataset.timestamps[i]
frame = StereoFrame(i, g2o.Isometry3d(), featurel, featurer, cam, timestamp=timestamp)

## images
imgL = frame.left.image
imgR = frame.right.image

## get features
featurel.extract()
featurer.extract()
kps_left, desps_left, idx_left = frame.left.get_unmatched_keypoints()
kps_right, desps_right, idx_right = frame.right.get_unmatched_keypoints()

matcher = frame.feature.matcher
matches = matcher.match(np.array(desps_left), np.array(desps_right))
goodMatchList = []
for m in matches:
    pt1 = kps_left[m.queryIdx].pt
    pt2 = kps_right[m.trainIdx].pt

    if (m.distance < 40 and
        abs(pt1[1] - pt2[1]) < 2.5 and
        abs(pt1[0] - pt2[0]) < 100):
        goodMatchList.append(m)

qIdx = goodMatchList[50].queryIdx
tIdx = goodMatchList[50].trainIdx

pt1 = kps_left[qIdx].pt
pt2 = kps_right[qIdx].pt


kps1 = [kps_left[m.queryIdx] for m in goodMatchList]
kps2 = [kps_right[m.trainIdx] for m in goodMatchList]

imgL = cv2.drawKeypoints(imgL, kps1,  None, color=(0, 0, 255))
imgR = cv2.drawKeypoints(imgR, kps2,  None, color=(0, 0, 255))

## matching
img3 = np.zeros((imgL.shape[0]*2, imgR.shape[1], imgR.shape[2]), dtype='uint8')
img3[0:imgL.shape[0]] = imgL
img3[imgL.shape[0]:] = imgR

for i in range(0, len(kps1[0:50])):
    kpsLeft = kps1[i].pt
    kpsRight = kps2[i].pt
    img3 = cv2.line(img3, (int(kpsLeft[0]), int(kpsLeft[1])),
                    (int(kpsRight[0]), int(kpsRight[1]) + 376), thickness=1, color=(255, 0, 0))


cv2.imshow('asdff', img3)
cv2.waitKey(100000)


