# pip libs
import cv2
import g2o
import os
import sys
import argparse
from threading import Thread

# custom libs
from Test.Test_Data import *
from Test.Test_Components import *

if __name__ == '__main__':
    params = ParamsKITTI()
    dataset = KITTIOdometry('/media/el/Data/DLData/KITTI/odom/dataset/sequences/00')

    # test = Test_Data(params, dataset)
    # test.showImage()
    # test.feature()
    testComp = Test_Components(params, dataset)
    testComp.showCameraParam()



























#
