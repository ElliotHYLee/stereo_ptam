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

class Test_Components:
    def __init__(self, params, dataset):
        self.dataset = dataset
        self.params = params

    def showCameraParam(self):
        dataset = self.dataset
        params = self.params
        cam = Camera(
            dataset.cam.fx, dataset.cam.fy, dataset.cam.cx, dataset.cam.cy,
            dataset.cam.width, dataset.cam.height,
            params.frustum_near, params.frustum_far,
            dataset.cam.baseline)
        print("K: ")
        print(cam.intrinsic)
        print("baseline: %.4f" % (cam.baseline))












        #
