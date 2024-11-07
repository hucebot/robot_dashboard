docker run  --privileged -v/dev/shm:/dev/shm  \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    --env HOME=/root \
    --env QT_X11_NO_MITSHM=1 \
    --env DISPLAY \
    --net host \
    -w /project \
    --volume `pwd`:/project \
    dashboard_ros1:latest "$@"
