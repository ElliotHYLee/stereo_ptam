# pip libs
import cv2
import g2o
import os
import sys
import argparse
from threading import Thread

# custom libs
from Components.Camera import Camera
from Components.StereoFrame import StereoFrame
from Feature.feature import ImageFeature
from Params.params import ParamsKITTI, ParamsEuroc
from Dataset.KITTIOdometry import KITTIOdometry
from Dataset.EuRoCDataset import EuRoCDataset

if __name__ == '__main__':
    params = ParamsKITTI()
    dataset = KITTIOdometry('/media/el/Data/DLData/KITTI/odom/dataset/sequences/00')
    img_0l = dataset.left[0]
    img_0r = dataset.right[0]
    featurel = ImageFeature(img_0l, params)
    featurer = ImageFeature(img_0r, params)
