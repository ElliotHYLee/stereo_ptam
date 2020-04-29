from sptam import *
import g2o
from threading import Thread
from Params.Camera import Camera
from Tracking.Frames.StereoFrame import StereoFrame
from Tracking.Frames.ImageFeature import ImageFeature
from Params.params import ParamsKITTI
from Dataset.KITTIOdometry import KITTIOdometry

if __name__ == '__main__':
    # parser = argparse.ArgumentParser()
    # parser.add_argument('--no-viz', action='store_true', help='do not visualize')
    # parser.add_argument('--dataset', type=str, help='dataset (KITTI/EuRoC)', default='KITTI')
    # parser.add_argument('--path', type=str, help='dataset path', default='path/to/your/KITTI_odometry/sequences/00')
    # args = parser.parse_args()

    ################
    # Prepare parameters and dataset
    ################

    # if args.dataset.lower() == 'kitti':
    #     params = ParamsKITTI()
    #     dataset = KITTIOdometry('~/Downloads/KITTI/sequences/00')
    # elif args.dataset.lower() == 'euroc':
        # params = ParamsEuroc()
        # dataset = EuRoCDataset(args.path)

    params = ParamsKITTI()
    dataset = KITTIOdometry('/mnt/F/KITTI/odom/dataset/sequences/00')

    ################
    # Prepare SPTAM main routine
    ################
    sptam = SPTAM(params)

    from Viewer.viewer import MapViewer
    viewer = MapViewer(sptam, params)

    ########
    # Prepare Camera Params
    ########
    cam = Camera(dataset.cam.fx, dataset.cam.fy, dataset.cam.cx, dataset.cam.cy,
                 dataset.cam.width, dataset.cam.height,
                 params.frustum_near, params.frustum_far,
                 dataset.cam.baseline)

    ## the for loop
    durations = []
    for i in range(0, 4000):#len(dataset)):#3000]:
        featurel = ImageFeature(dataset.left[i], params)
        featurer = ImageFeature(dataset.right[i], params)
        timestamp = dataset.timestamps[i]

        time_start = time.time()

        ## Do we really need the below? The init and join takes more time

        # while extracting right feature,
        t = Thread(target=featurer.extract)
        t.start()
        # extract left feature from the main thread
        featurel.extract()
        t.join()

        frame = StereoFrame(i, g2o.Isometry3d(), featurel, featurer, cam, timestamp=timestamp)

        if not sptam.is_initialized():
            sptam.initialize(frame)
        else:
            sptam.track(frame)


        duration = time.time() - time_start
        durations.append(duration)
        print('duration', duration)
        print()
        print()


        viewer.update()

    print('num frames', len(durations))
    print('num keyframes', len(sptam.graph.keyframes()))
    print('average time', np.mean(durations))


    sptam.stop()
    viewer.stop()
