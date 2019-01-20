# pip libs
import cv2
import g2o
import os
import sys
import argparse
from threading import Thread

# custom libs
from Test.Test_ImgData import *
from Test.Test_Components import *

def test_camParam(params, dataset):
    testComp = Test_Components(params, dataset)
    testComp.showCameraParam()

def test_data(params, dataset):
    test = Test_ImgData(params, dataset)
    #test.showImage()
    test.feature()
    

if __name__ == '__main__':
    params = ParamsKITTI()
    dataset = KITTIOdometry('/media/el/Data/DLData/KITTI/odom/dataset/sequences/00')

    # cam param test
    test_camParam(params, dataset)

    # img data test
    test_data(params, dataset)




























#
