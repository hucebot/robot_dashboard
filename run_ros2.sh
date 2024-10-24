xhost +local:docker

set -x
docker run  --privileged \
    -v /dev/shm:/dev/shm \
    -v /tmp/.X11-unix/:/tmp/.X11-unix -it \
    -e HOME=/root \
    -e QT_X11_NO_MITSHM=1 \
    -e DISPLAY \
    --net host \
    --ipc host \
    --pid host \
    -w /project \
    -v `pwd`:/project \
    dashboard_ros2:latest "$@" #python3 dashboard_video.py configs/teleop_tiago.yaml layouts/layout_teleop.yaml
