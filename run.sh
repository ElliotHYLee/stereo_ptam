docker run -it --rm \
--name con_try_1 \
-v ~/Github/stereo_ptam/src:/home/src \
-v /mnt/KITTI:/home/src/KITTI \
--net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
hylee101001/slam:sptam_env run.sh
