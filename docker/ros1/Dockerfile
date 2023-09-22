# inspired by https://github.com/hsp-panda/dockerfiles-panda/tree/main/ros_realsense
FROM ros:noetic

LABEL maintainer="jean-baptiste.mouret@inria.fr"

ENV ROS_DISTRO noetic

ENV DEBIAN_FRONTEND="noninteractive" TZ="Europe/Paris"

RUN apt -q -qq update && \
  DEBIAN_FRONTEND=noninteractive apt install -y --allow-unauthenticated \
  ros-${ROS_DISTRO}-jsk-tools \
  ros-${ROS_DISTRO}-controller-manager

RUN apt -q -qq update && apt install -y --allow-unauthenticated \
  python3-catkin \
  python3-catkin-pkg \
  python3-catkin-tools \
  python3-pyqt5 \
  python3-matplotlib \
  python3-pyqt5.qtmultimedia \
  python3-psutil \
  python3-pip \
  pyqt5-dev-tools


RUN  apt -q -qq update && apt install -y --allow-unauthenticated \
  fping \
  git \
  ntpdate \
  wireless-tools



# switch to python3
RUN apt install python-is-python3

RUN pip3 install numpy pyqtgraph


# init catkin workspace
#RUN . /opt/ros/${ROS_DISTRO}/setup.sh && mkdir -p /catkin_ws/src && cd /catkin_ws/ && catkin build

#RUN rosdep update


RUN touch /root/.bashrc \   
    && echo "export ROS_PYTHON_VERSION=3" >> /root/.bashrc \
    && echo "source /opt/ros/noetic/setub.bash"  >> /root/.bashrc

RUN cd /root && git clone https://github.com/hucebot/robot_dashboard


CMD ["cd /project && python3 dashboard.py"]
