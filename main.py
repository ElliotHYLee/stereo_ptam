from sptam import *


if __name__ == '__main__':
    import cv2
    import g2o

    import os
    import sys
    import argparse

    from threading import Thread

    from Components.Camera import Camera
    from Components.StereoFrame import StereoFrame

    from Feature.feature import ImageFeature
    from params import ParamsKITTI, ParamsEuroc
    from dataset import KITTIOdometry, EuRoCDataset


    parser = argparse.ArgumentParser()
    parser.add_argument('--no-viz', action='store_true', help='do not visualize')
    parser.add_argument('--dataset', type=str, help='dataset (KITTI/EuRoC)', default='KITTI')
    parser.add_argument('--path', type=str, help='dataset path', default='path/to/your/KITTI_odometry/sequences/00')
    args = parser.parse_args()

    if args.dataset.lower() == 'kitti':
        params = ParamsKITTI()
        dataset = KITTIOdometry(args.path)
    elif args.dataset.lower() == 'euroc':
        params = ParamsEuroc()
        dataset = EuRoCDataset(args.path)

    sptam = SPTAM(params)

    visualize = not args.no_viz
    if visualize:
        from viewer import MapViewer
        viewer = MapViewer(sptam, params)

    cam = Camera(
        dataset.cam.fx, dataset.cam.fy, dataset.cam.cx, dataset.cam.cy,
        dataset.cam.width, dataset.cam.height,
        params.frustum_near, params.frustum_far,
        dataset.cam.baseline)

    durations = []
    for i in range(len(dataset))[:3000]:#3000]:
        featurel = ImageFeature(dataset.left[i], params)
        featurer = ImageFeature(dataset.right[i], params)
        timestamp = dataset.timestamps[i]

        time_start = time.time()

        t = Thread(target=featurer.extract)
        t.start()
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

        if visualize:
            viewer.update()

    print('num frames', len(durations))
    print('num keyframes', len(sptam.graph.keyframes()))
    print('average time', np.mean(durations))


    sptam.stop()
    if visualize:
        viewer.stop()
