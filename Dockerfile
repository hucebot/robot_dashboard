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
  ros-${ROS_DISTRO}-controller-manager

RUN apt -q -qq update && apt install -y --allow-unauthenticated \
  python3-catkin \
  python3-catkin-pkg \
  python3-catkin-tools \
  python3-pyqt5 \
  python3-matplotlib \
  python3-pyqt5.qtmultimedia

RUN  apt -q -qq update && apt install -y --allow-unauthenticated \
  fping \
  git \
  qtcreator \
  pyqt5-dev-tools \
  ntpdate \
  python3-psutil \
  python3-pip

RUN  apt -q -qq update && apt install -y --allow-unauthenticated \
  gstreamer1.0-plugins-good \ 
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-rtp \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  python3-gst-1.0 \
  net-tools curl


# switch to python3
RUN apt install python-is-python3

RUN pip3 install numpy

RUN pip3 install pyqtgraph

# init catkin workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && mkdir -p /catkin_ws/src && cd /catkin_ws/ && catkin build

RUN rosdep update


 RUN touch /root/.bashrc \   
    && echo "export ROS_PYTHON_VERSION=3" >> /root/.bashrc

RUN cd /root && git clone https://github.com/hucebot/robot_dashboard



### ROS 2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


RUN python3 -m pip install -U colcon-common-extensions vcstool

RUN mkdir -p ~/ros2_humble/src \
 && cd ~/ros2_humble \
 && vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src


RUN apt update && apt-get install -y libignition-math6-dev
RUN apt update && apt install -y python3-catkin-pkg-modules
RUN apt upgrade -y
RUN rosdep update && cd ~/ros2_humble/ && rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

RUN cd ~/ros2_humble/ && colcon build --symlink-install
RUN echo "source /root/ros2_humble/install/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc


RUN  apt -q -qq update && apt install -y --allow-unauthenticated \
  wireless-tools


CMD ["cd /project && python3 dashboard.py"]
