import g2o
import numpy as np

def test_g2o():
    ## g2o.Isometry3d() returns an object containing rotation and translation
    pose = g2o.Isometry3d()
    vector = np.array([0.5, 0, 0])
    pos = pose*vector ## find out what g2o.Isometry3d and numpy multiplication
    res_g2o = g2o.Isometry3d(pose.orientation(), pos)
    res_np = g2o.Isometry3d(pose.orientation(), vector)

    print(res_g2o.matrix())
    print(res_np.matrix())
    print(res_g2o.matrix() == res_np.matrix())

if __name__ == '__main__':
    test_g2o()




