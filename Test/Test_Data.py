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

class Test_Data:
    def __init__(self, params, dataset):
        self.params = params
        self.dataset = dataset
        self.img_0l = dataset.left[0]
        self.img_0r = dataset.right[0]
    def showImage(self):
        cv2.imshow('left', self.img_0l)
        cv2.imshow('right', self.img_0r)
        cv2.waitKey(3*1000)

    def feature(self):
        featurel = ImageFeature(self.img_0l, self.params)
        featurer = ImageFeature(self.img_0r, self.params)










        #
