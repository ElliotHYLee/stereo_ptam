# pip libs
import cv2
from threading import Thread
import time

# custom libs
from Frames.ImageFeature import ImageFeature


class Test_ImgData:
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

        start = time.time()
        featurel.extract()
        featurer.extract()
        end = time.time()
        print(end - start)

        start = time.time()
        t = Thread(target=featurel.extract)
        t.start()
        featurer.extract()
        t.join()
        end = time.time()
        print(end - start)








        #
