# needs to be before QT
from gstreamer_feed import GStreamerFeed

import time
import numpy as np

from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtGui
import sys
#import cv2
import numpy as np
from collections import deque
import yaml
import dark_style



class GstreamerThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    ready = pyqtSignal(int)
    

    def __init__(self, conf):
        super().__init__()
        self.conf = conf

    def run(self):
        self.feed = GStreamerFeed(self.conf)
        self.feed.start()
        self.__run = True
        self.ready.emit(2)
        while True:
            if self.feed.isFrameReady() and self.__run:
                np_img = self.feed.getFrame()
                self.change_pixmap_signal.emit(np_img)
            if not self.feed.check_messages():
                self.ready.emit(2)
            elif self.feed.playing:
                self.ready.emit(1)
            else:
                self.ready.emit(2)



    def taskStop(self):
        self.__run = False

    def taskStart(self):
        self.__run = True



class GstreamerWindow(QWidget):
    # to send data to plots
    new_fps_data_signal = pyqtSignal(float)
    new_bitrate_data_signal = pyqtSignal(float)
    new_jitter_data_signal =  pyqtSignal(float)
    new_delay_data_signal =  pyqtSignal(float)

    def __init__(self, conf, dashboard):
        super().__init__()
        self.conf = conf
        self.setWindowTitle("Camera")
        self.dashboard = dashboard

        # create the label that holds the image
        self.image_label = QLabel(self)
        self.image_label.setMinimumSize(1, 1)
        #self.image_label.resize(self.width(), self.height())
        self.image_label.setText("<center><h1>Waiting for video stream...</h1></center>")
        
        
        vbox = QVBoxLayout()
        hbox = QHBoxLayout()
        self.setLayout(hbox)
        self.slider_tilt = QSlider(Qt.Vertical)
        self.slider_tilt.setMinimum(-50)
        self.slider_tilt.setMaximum(50)
        
        hbox.addWidget(self.slider_tilt)


        vbox.addWidget(self.image_label)
        self.slider_pan = self.slider_pan = QSlider(Qt.Horizontal)
        self.slider_pan.setMinimum(-50)
        self.slider_pan.setMaximum(50)

        vbox.addWidget(self.slider_pan)
        hbox.addLayout(vbox)


        self.slider_pan.valueChanged.connect(self.dashboard.thread_ros2.set_pan)
        self.slider_tilt.valueChanged.connect(self.dashboard.thread_ros2.set_tilt)

        self.thread = GstreamerThread(self.conf)
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()

        self.time_list = []        
        self.new_fps_data_signal.connect(dashboard.plots['fps'].new_data)

        self.window_size = self.size()

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        qt_img = self.convert_cv_qt(cv_img)
        self.image_label.setPixmap(qt_img)
        self.time_list.append(time.time())
        if len(self.time_list) == self.conf['plot_videostream_period']:
            fps = len(self.time_list) / (self.time_list[-1] -  self.time_list[0])
            self.time_list = []
            self.new_fps_data_signal.emit(fps)

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(cv_img.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        # in portrait we scaled to height, in landscape to width
        if self.width() > self.height():
            p = convert_to_Qt_format.scaledToWidth(self.width())
        else:
            p = convert_to_Qt_format.scaledToHeight(self.height())
        return QPixmap.fromImage(p)
    
