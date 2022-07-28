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
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import *
from controller_manager_msgs.utils\
    import ControllerLister, ControllerManagerLister,\
    get_rosparam_controller_names
from diagnostic_msgs.msg import DiagnosticArray

import socket


# Local Module Imports
import dashboard_ui

# a nice green for LEDS
GREEN = '#66ff00'


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
        ip = os.environ["ROS_MASTER_URI"].split('//')[1].split(':')[0]
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
        self.ros_pubs = []
        self.label_ros_uri.setText("["  + os.environ["ROS_MASTER_URI"] + "]")
        try:
            if rosgraph.is_master_online():
                self.ros_ok = True
                if self.ros_master == None:
                    self.ros_master = rosgraph.Master('/rostopic')
                self.ros_pubs, self.ros_subs = rostopic.get_topic_list(master=self.ros_master)
            else:
                self.ros_ok = False     
        except Exception as e:
            self.ros_ok = False
        if self.ros_ok:
            self.led_color(self.led_ros, GREEN)
        else: # ros is down! let' s reinit everything
            print("ROS is down!")
            self.reinit()
        if self.ros_ok and self.topic_diag == None:
            rospy.init_node('dashboard', anonymous=True)
            self.topic_diag = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.diag_cb)
            print("subscribed to /diagnostics_agg")

    def reinit(self):
        print("reinit")
        self.led_color(self.led_ros, 'red')
        self.led_color(self.led_controller, 'red')
        self.emergency = True
        self.controller_lister = None
        self.topic_diag = None
        self.ros_master = None
        self.topic_diag = None
        self.led_motors = {}
        self.motors = {}

    def update_ros_control(self):
        if self.ros_ok:
            try:
                if self.controller_lister == None:
                    rospy.wait_for_service('/controller_manager/list_controllers',timeout=0.05)
                    self.controller_lister = ControllerLister('/controller_manager')
                self.controller_list = self.controller_lister()
                self.controller_states = {}
                for i in self.led_controllers.keys():
                    self.controller_states[i] = False
                for c in self.controller_list:
                    if not c.name in self.led_controllers.keys():
                        self.led_controllers[c.name] = QtWidgets.QRadioButton(c.name, self.centralwidget)
                        self.led_controllers[c.name].setObjectName(c.name)
                        self.verticalLayout.insertWidget(len(self.verticalLayout)-1, self.led_controllers[c.name])
                    if c.state == 'running':
                        self.controller_states[c.name] = True
                for c in self.led_controllers.keys():
                    if self.controller_states[c]:
                        self.led_color(self.led_controllers[c], GREEN)
                    else:
                        self.led_color(self.led_controllers[c], 'red')
                self.ros_control_ok = True
                self.led_color(self.led_controller, GREEN)
            except:
                self.ros_control_ok = False
                self.led_color(self.led_controller, 'red')
                return
        else:
            self.led_color(self.led_controller, 'red')

    def update_diagnostics(self):
        if self.emergency:
            self.led_color(self.led_emergency, 'red')
        else:
            self.led_color(self.led_emergency, GREEN)
        if len(self.led_motors) == 0:
            for k in self.motors.keys():
                self.led_motors[k] = QtWidgets.QRadioButton(k, self.centralwidget)
                self.led_motors[k].setObjectName(k)
                self.layout_motors.insertWidget(len(self.layout_motors) - 1, self.led_motors[k])
        for k in self.led_motors:
            if self.motors[k]:
                self.led_color(self.led_motors[k], GREEN)
            else:
                self.led_color(self.led_motors[k], 'red')

    def led_color(self, b, color):
        b.setStyleSheet("QRadioButton::indicator {width: 14px; height: 14px; border-radius: 7px;} QRadioButton::indicator:unchecked { background-color:" + color + "}")

    def diag_cb(self, msg):
        for m in msg.status:
            if m.name == '/Hardware/Battery':
                self.label_battery.setText("Battery: " + m.values[0].value)
            elif m.name == "/Hardware/Control PC/Load Average":
                print("CPU:", m.values[1].value)# load avg 1-min
            elif m.name == "/Hardware/Control PC/Emergency Button":
                if(m.message == ''):
                    self.emergency = False
                else:
                    self.emergency = True                    
            elif "/Hardware/Motor/"  in m.name:
                n = m.name.split("/")[-1]
                if m.message:
                    self.motors[n] = False
                else:
                    self.motors[n] = True

    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUi(self)

        self.label_ros_uri.setStyleSheet("font-weight: bold")
        self.reinit()

        socket.setdefaulttimeout(0.05)# give 50ms to answer


        # red leds
        self.leds = [self.led_robot, self.led_ros, self.led_controller]
        for l in self.leds:
            self.led_color(l, "red")
            l.setDisabled(True)

        # ping
        self.ping_queue = deque([], maxlen = 50)
        self.timer_ping = QTimer()
        self.timer_ping.timeout.connect(self.update_ping)
        self.ros_ok = False
        self.timer_ping.start(1000)

        # ros
        self.timer_ros = QTimer()
        self.timer_ros.timeout.connect(self.update_ros_topics)
        self.timer_ros.start(2000)
        
        # control manager
        self.led_controllers = {}
        self.timer_ros_control = QTimer()
        self.timer_ros_control.timeout.connect(self.update_ros_control)
        self.ros_control_ok = False
        self.timer_ros_control.start(3000)

        # diagnostics (motors, load, etc.)
        self.timer_diag = QTimer()
        self.timer_diag.timeout.connect(self.update_diagnostics)
        self.timer_diag.start(100)# can be fast because only update GUI

def main():
    app = QtWidgets.QApplication(sys.argv)
    dark_style(app)
    form = Dashboard()
    form.show()
    app.exec_()


if __name__ == '__main__':
    main()
