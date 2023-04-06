import sys
import numpy as np
from PyQt5 import QtGui
from datetime import datetime
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtGui
import sys
import cv2
import numpy as np

from gstreamer_feed import GStreamerFeed


class GstreamerThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    ready = pyqtSignal(int)

    def __init__(self, launch:str=''):
        super().__init__()
        self.launch = launch

    def run(self):
        self.cap = GStreamerFeed(self.launch)
        self.cap.start()
        self.__run = True
        first  = True
        self.ready.emit(2)
        while True:
            if self.cap.isFrameReady() and self.__run:
                np_img = self.cap.getFrame()
                self.change_pixmap_signal.emit(np_img)
                if first:
                    self.ready.emit(1)
                    first = False

    def taskStop(self):
        self.__run = False

    def taskStart(self):
        self.__run = True



class GstreamerWindow(QWidget):
    def __init__(self, conf):
        super().__init__()
        self.conf = conf
        self.setWindowTitle("Camera")

        # create the label that holds the image
        self.image_label = QLabel(self)
        self.image_label.resize(self.width(), self.height())

        vbox = QVBoxLayout()
        vbox.addWidget(self.image_label)
        self.setLayout(vbox)

        self.thread = GstreamerThread(self.conf['gstreamer_launch'])
        self.thread.change_pixmap_signal.connect(self.update_image)
        # start the thread
        self.thread.start()


    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        qt_img = self.convert_cv_qt(cv_img)
        self.image_label.setPixmap(qt_img)
    
    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_BGR888)
       # p = convert_to_Qt_format.scaled(self.disply_width, self.display_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(convert_to_Qt_format)
    