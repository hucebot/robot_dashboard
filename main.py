from PyQt5 import QtWidgets
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtMultimedia import (QCamera, QCameraImageCapture,
                                QImageEncoderSettings, QMediaMetaData,
                                QMediaRecorder, QMultimedia,
                                QVideoEncoderSettings)
from PyQt5.QtWidgets import *

import sys
import subprocess 
from collections import deque
import rostopic
import rosgraph

# Local Module Imports
import dashboard_ui
  
def dark_style(app):
    # set a dark theme to the app!
    app.setStyle('fusion')
    dark_palette = QPalette()

    dark_palette.setColor(QPalette.Window, QColor(0, 0, 0))
    dark_palette.setColor(QPalette.WindowText, Qt.white)
    dark_palette.setColor(QPalette.Base, QColor(25, 25, 25))
    dark_palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ToolTipBase, Qt.white)
    dark_palette.setColor(QPalette.ToolTipText, Qt.white)
    dark_palette.setColor(QPalette.Text, Qt.white)
    dark_palette.setColor(QPalette.Button, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.Disabled, QPalette.Button, QColor(30,30,30));
    dark_palette.setColor(QPalette.ButtonText, Qt.white)
    dark_palette.setColor(QPalette.Disabled, QPalette.ButtonText, QColor(100, 100, 100));
    dark_palette.setColor(QPalette.BrightText, Qt.red)
    dark_palette.setColor(QPalette.Link, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.Disabled, QPalette.Highlight, QColor(100, 100, 100))

    dark_palette.setColor(QPalette.HighlightedText, Qt.black)

    app.setPalette(dark_palette)

    app.setStyleSheet(
        "QToolTip { color: #ffffff; background-color: #2a82da; border: 1px solid white; }")
    # end of dark theme

class Dashboard(QtWidgets.QMainWindow, dashboard_ui.Ui_RobotDashBoard):
    def update_ping(self):
        ip = self.drop_ip.currentText()
        try:
            c = subprocess.check_output(["ping", "-c", "1", "-t", "1", ip])
            c = c.split(b'\n')[-3:]
            t = c[1].split(b"=")[1].split(b"/")[0]
            p = float(t)
            self.ping_queue.append(p)
            self.plot_ping.set_data(self.ping_queue)
            self.led_color(self.led_robot, '#66ff00')
        except:
            self.led_color(self.led_robot, 'red')

    def led_color(self, b, color):
        b.setStyleSheet("QRadioButton::indicator {width: 14px; height: 14px; border-radius: 7px;} QRadioButton::indicator:unchecked { background-color:" + color + "}")

    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUi(self)

        # ip
        #self.label_ip.setText("192.168.1.1")
        self.drop_ip.addItem("192.168.1.1")
        self.drop_ip.addItem("tiago-61c")
        self.drop_ip.addItem("talos-5c")

        # red leds
        self.leds = [self.led_robot, self.led_ros, self.led_motors, self.led_controller, self.led_joystick, self.led_geomagic]
        for l in self.leds:
            self.led_color(l, "red")
            l.setDisabled(True)

        # ping
        self.ping_queue = deque([], maxlen = 100)
        self.timer_ping = QTimer()
        self.timer_ping.timeout.connect(self.update_ping)
        self.timer_ping.start(250)


def main():
    app = QtWidgets.QApplication(sys.argv)
    dark_style(app)
    form = Dashboard()
    form.show()
    app.exec_()


if __name__ == '__main__':
    main()
