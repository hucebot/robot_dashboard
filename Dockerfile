# inspired by https://github.com/hsp-panda/dockerfiles-panda/tree/main/ros_realsense
FROM ros:noetic

LABEL maintainer="jean-baptiste.mouret@inria.fr"

ENV ROS_DISTRO noetic

ENV DEBIAN_FRONTEND="noninteractive" TZ="Europe/Paris"

RUN apt -q -qq update && \
  DEBIAN_FRONTEND=noninteractive apt install -y --allow-unauthenticated \
  ros-${ROS_DISTRO}-jsk-tools \
  ros-${ROS_DISTRO}-rgbd-launch \
  ros-${ROS_DISTRO}-image-transport-plugins \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-controller-manager \
  python3-catkin \
  python3-catkin-pkg \
  python3-catkin-tools \
  python3-pyqt5 \
  python3-matplotlib \
  python3-pyqt5.qtmultimedia \
  fping \
  git \
  qtcreator \
  pyqt5-dev-tools \
  ntpdate \
  python3-psutil \
  python3-pip \
  gstreamer1.0-plugins-good \ 
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-rtp \
  gstreamer1.0-plugins-ugly


# switch to python3
RUN apt install python-is-python3

RUN pip3 install pyqtgraph

# init catkin workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && mkdir -p /catkin_ws/src && cd /catkin_ws/ && catkin build

RUN rosdep update


 RUN touch /root/.bashrc \   
    && echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc \
    && echo "export ROS_PYTHON_VERSION=3" >> /root/.bashrc

RUN cd /root && git clone https://github.com/hucebot/robot_dashboard

# Set entrypoint

#COPY ./ros_entrypoint.sh /
#ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
