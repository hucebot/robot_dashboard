FROM ros:humble-ros-base

LABEL maintainer="jean-baptiste.mouret@inria.fr"
ENV DEBIAN_FRONTEND="noninteractive" TZ="Europe/Paris"

ENV ROS_DISTRO humble



RUN apt -q -qq update && apt install -y --allow-unauthenticated \
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
wireless-tools


RUN  apt -q -qq update && apt install -y --allow-unauthenticated \
  gstreamer1.0-plugins-good \ 
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-rtp \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-tools \
  python3-gst-1.0 \
  net-tools curl


# switch to python3
RUN apt install python-is-python3
RUN pip3 install numpy
RUN pip3 install pyqtgraph
RUN cd /root && git clone https://github.com/hucebot/robot_dashboard

### 6d mouse
RUN pip3 install pyspacemouse
RUN pip3 install git+https://github.com/bglopez/python-easyhid.git
RUN pip3 install hidapi
RUN pip3 install scipy
RUN apt-get -yy install libhidapi-dev

### ROS 2
RUN python3 -m pip install -U colcon-common-extensions vcstool
RUN apt update && apt-get install -y libignition-math6-dev


RUN echo "source /opt/ros/humble/setup.bash"  >> /root/.bashrc

CMD ["/bin/bash"]
