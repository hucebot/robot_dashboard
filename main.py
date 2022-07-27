from PyQt5 import QtWidgets
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import sys, os
import subprocess 
from collections import deque

import rostopic
import rosgraph
import rospy


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

GREEN = '#66ff00'
class Dashboard(QtWidgets.QMainWindow, dashboard_ui.Ui_RobotDashBoard):
    def update_ping(self):
        ip = self.drop_ip.currentText()
        try:
            c = subprocess.check_output(["fping", "-c1", "-t60", ip],stderr=subprocess.STDOUT)
            t = c.split(b" ")[5]
            p = float(t)
            self.ping_queue.append(p)
            self.led_color(self.led_robot, GREEN)
            self.plot_ping.error(False)

        except:
            self.led_color(self.led_robot, 'red')
            self.plot_ping.error(True)
        self.plot_ping.set_data(self.ping_queue)


    def update_ros_topics(self):
        ip = self.drop_ip.currentText()
        os.environ["ROS_MASTER_URI"] = "http://" + ip +":11311"
        self.ros_pubs = []
        try:
            self.ros_pubs, self.ros_subs = rostopic.get_topic_list(master=self.ros_master)
            self.led_color(self.led_ros, GREEN)
            self.ros_ok = True
        except:
            self.led_color(self.led_ros, 'red')
            self.ros_ok = False

    def update_ros_control(self):
        if self.ros_ok:
            try:
                rospy.wait_for_service('/controller_manager/list_controllers', 0.1)
                self.ros_control_ok = True
            except rospy.ROSException:
                self.ros_control_ok = False
                return
            
            

    def led_color(self, b, color):
        b.setStyleSheet("QRadioButton::indicator {width: 14px; height: 14px; border-radius: 7px;} QRadioButton::indicator:unchecked { background-color:" + color + "}")

    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUi(self)

        # ip
        self.drop_ip.addItem("localhost")
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
        self.ros_ok = False
        self.timer_ping.start(250)

        # ros
        self.ros_master = rosgraph.Master('/rostopic')
        self.timer_ros = QTimer()
        self.timer_ros.timeout.connect(self.update_ros_topics)
        self.timer_ros.start(500)
        
        # control manager
        self.timer_ros_control = QTimer()
        self.timer_ros_control.timeout.connect(self.update_ros_control)
        self.ros_control_ok = False
        self.timer_ros_control.start(500)
def main():
    app = QtWidgets.QApplication(sys.argv)
    dark_style(app)
    form = Dashboard()
    form.show()
    app.exec_()


if __name__ == '__main__':
    main()
