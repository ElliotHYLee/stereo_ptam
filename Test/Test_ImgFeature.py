# pip libs
import cv2
import g2o
import os
import sys
import argparse
from threading import Thread
import time

# custom libs
from Components.Camera import Camera
from Components.StereoFrame import StereoFrame
from Feature.ImageFeature import ImageFeature
from Params.params import ParamsKITTI, ParamsEuroc
from Dataset.KITTIOdometry import KITTIOdometry
from Dataset.EuRoCDataset import EuRoCDataset


class Test_ImgFeature:
    def __init__(self, params, dataset):
        self.params = params
        self.dataset = dataset
        self.img_0l = dataset.left[0]
        self.img_0r = dataset.right[0]

    def test_extractFeature(self):
        featurel = ImageFeature(self.img_0l, self.params)
        featurer = ImageFeature(self.img_0r, self.params)

        ## compare thread and series left and right feature extraction
        # 1. case1: series extraction
        start = time.time()
        featurel.extract()
        featurer.extract()
        end = time.time()
        print(end - start)

        # 2. case2: parallel extraction
        start = time.time()
        t = Thread(target=featurel.extract)
        t.start()
        featurer.extract()
        t.join()
        end = time.time()
        print(end - start)

        #

    def test_matchFeature(self):
        imf_l = ImageFeature(self.img_0l, self.params)
        imf_l.extract()
        imf_r = ImageFeature(self.img_0r, self.params)
        imf_r.extract()
        #imf_l.draw_keypoints('keypoints', delay=1000*1)

        kps_l, desps_l, idx_l = imf_l.get_unmatched_keypoints()
        kps_r, desps_r, idx_r = imf_r.get_unmatched_keypoints()

        matches = imf_l.row_match(kps_l, desps_l, kps_r, desps_r)

        print(matches)

        # print(kps)
        # print(desps)
        # print(idx)


