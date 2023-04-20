import time
import numpy as np
from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtGui
import sys
import cv2
import numpy as np
from collections import deque

from gstreamer_feed import GStreamerFeed


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
        self.image_label.resize(self.width(), self.height())
        self.image_label.setText("<center><h1>Waiting for video stream...</h1></center>")
        self.img_width = self.width()
        self.img_height = self.height()
        vbox = QVBoxLayout()
        vbox.addWidget(self.image_label)
        self.setLayout(vbox)

        self.thread = GstreamerThread(self.conf)
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()

        self.time_list = []        
        self.new_fps_data_signal.connect(dashboard.plot_fps.new_data)

        self.bitrate_queue = deque([], self.conf['plot_videostream_period'])
        self.new_bitrate_data_signal.connect(dashboard.plot_bitrate.new_data)

        self.jitter_queue = deque([], self.conf['plot_videostream_period'])
        self.new_jitter_data_signal.connect(dashboard.plot_jitter.new_data)
        
        self.new_delay_data_signal.connect(dashboard.plot_delay.new_data)


    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        qt_img = self.convert_cv_qt(cv_img)
        self.time_list.append(time.time())
        self.bitrate_queue.append(self.thread.feed.bitrate / 1000000)
        # jitter is in ns -> we convert to ms
        self.jitter_queue.append(self.thread.feed.jitter / 1e6)
        self.image_label.setPixmap(qt_img)
        if len(self.time_list) == self.conf['plot_videostream_period']:
            fps = len(self.time_list) / (self.time_list[-1] -  self.time_list[0])
            bitrate = np.mean(self.bitrate_queue)
            jitter = np.mean(self.jitter_queue)
            self.time_list = []
            self.new_fps_data_signal.emit(fps)
            self.new_bitrate_data_signal.emit(bitrate)
            self.new_jitter_data_signal.emit(jitter)
            self.new_delay_data_signal.emit(self.thread.feed.delay)

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_BGR888)
        p = convert_to_Qt_format.scaled(self.img_width, self.img_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)#convert_to_Qt_format)
    