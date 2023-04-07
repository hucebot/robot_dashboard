#!/usr/bin/python
from gstreamer_window import GstreamerWindow
from PyQt5.QtWidgets import QStyleFactory
from PyQt5.QtWidgets import QApplication, QDesktopWidget
import dashboard_ui
from PyQt5 import QtWidgets
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import psutil

import sys
import os
import time
import subprocess
from collections import deque
import socket
import numpy as np
import yaml
import pyqtgraph as pg

# a nice green for LEDS
GREEN = '#66ff00'

# we can start without ROS (to debug in a train, on Mac, etc.)
USE_ROS = True
try:
    import rostopic
    import rosgraph
    import rospy
    from controller_manager_msgs.msg import ControllerState
    from controller_manager_msgs.srv import *
    from controller_manager_msgs.utils\
        import ControllerLister, ControllerManagerLister,\
        get_rosparam_controller_names
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import Float64MultiArray
    from talos_controller_msgs.msg import float64_array
except Exception as e:
    print("WARNING, ROS disabled: CANNOT IMPORT ROS")
    print(e)
    USE_ROS = False


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
    dark_palette.setColor(
        QPalette.Disabled, QPalette.Button, QColor(30, 30, 30))
    dark_palette.setColor(QPalette.ButtonText, Qt.white)
    dark_palette.setColor(
        QPalette.Disabled, QPalette.ButtonText, QColor(100, 100, 100))
    dark_palette.setColor(QPalette.BrightText, Qt.red)
    dark_palette.setColor(QPalette.Link, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    dark_palette.setColor(
        QPalette.Disabled, QPalette.Highlight, QColor(100, 100, 100))

    dark_palette.setColor(QPalette.HighlightedText, Qt.black)

    app.setPalette(dark_palette)

    app.setStyleSheet(
        "QToolTip { color: #ffffff; background-color: #2a82da; border: 1px solid white; }")
    # end of dark theme

# all the plots follow the same style
# this could be a custom widget


class Plot:
    def __init__(self, widget, min=None, max=None):
        self.widget = widget
        self.widget.showGrid(x=True, y=True)
        self.line_bg = self.widget.plot(
            [], [], pen=pg.mkPen(color=(102, 255, 0, 128), width=8))
        self.line = self.widget.plot([], [], pen=pg.mkPen(color=(102, 255, 0)))
        self.error = False
        if min != None:
            assert (max != None)
            self.widget.setYRange(min, max)

    def update(self, data):
        self.line_bg.setData(np.arange(len(data)), data)
        self.line.setData(np.arange(len(data)), data)

# ping is blocking and we do not want that


class PingThread(QThread):
    new_data = pyqtSignal(float)
    ok = pyqtSignal(int)
    ping = 0

    def __init__(self, conf):
        super().__init__()
        self.conf = conf

    def run(self):
        while True:
            try:
                ip = self.conf['robot_ip']
                t = "-t" + str(self.conf['ping_timeout'])
                c = subprocess.check_output(
                    ["fping", "-c1", t, ip], stderr=subprocess.STDOUT)
                t = c.split(b" ")[5]
                self.ping = float(t)
                self.ok.emit(1)
            except Exception as e:
                print(e)
                self.ping = 100
                self.ok.emit(0)
            self.new_data.emit(self.ping)
            time.sleep(self.conf['ping_period'])


class Dashboard(QtWidgets.QMainWindow, dashboard_ui.Ui_RobotDashBoard):
    new_data_net_sent_signal = pyqtSignal(float)
    new_data_net_recv_signal = pyqtSignal(float)
   
    def update_network_stats(self):
        net_io_counters = psutil.net_io_counters(pernic=True)
        iname = self.conf['local_interface_name']
        if iname in net_io_counters:
            stats = net_io_counters[iname]
            t = time.time()
            b = (stats.bytes_sent, stats.bytes_recv, t)
            if len(self.prev_network_stats) != 0:
                prev_s, prev_r, prev_t = self.prev_network_stats
                d = t - prev_t
                sent = (stats.bytes_sent - prev_s) / d / (1024**2)
                recv = (stats.bytes_recv - prev_r) / d / (1024**2)
                self.new_data_net_sent_signal.emit(sent)
                self.new_data_net_recv_signal.emit(recv)
            self.prev_network_stats = b
        else:
            print("Interface not found")

    def update_ros_topics(self):
        self.update_ping()
        if self.robot_ok == False:
            self.ros_ok = False
            self.reinit()
            return
        self.ros_pubs = []
        os.environ["ROS_MASTER_URI"] = 'http://' + self.conf['robot_ip'] + ':11311'
        self.label_ros_uri.setText("[" + os.environ["ROS_MASTER_URI"] + "]")
        prev_ros = self.ros_ok
        if self.ros_master == None:
            self.ros_master = rosgraph.Master(
                '/rostopic', os.environ["ROS_MASTER_URI"])
        try:
            if self.ros_master.is_online():
                self.ros_ok = True
                self.ros_pubs, self.ros_subs = rostopic.get_topic_list(
                    master=self.ros_master)
                ros_pubs_names = [item[0] for item in self.ros_pubs]
                for i in self.topic_list:
                    if i in ros_pubs_names:
                        self.led_color(self.led_topics[i], GREEN)
                    else:
                        self.led_color(self.led_topics[i], 'red')

            else:
                print("master is offline")
                self.ros_ok = False
        except Exception as e:
            self.ros_ok = False
            print("Ros: exception", e)

        if self.ros_ok:
            self.led_color(self.led_ros, GREEN)
        else:  # ros is down! let' s reinit everything
            print("ROS is down!")
            self.reinit()

        # diagnostics
        if self.ros_ok and self.topic_diag == None:
            rospy.init_node('dashboard', anonymous=True)
            self.topic_diag = rospy.Subscriber(
                "/diagnostics_agg", DiagnosticArray, self.diag_cb)

    def reinit(self):
        frame = inspect.stack()[1]
        print("reinit, called from:",
              frame[3] + " line:", frame.frame.f_lineno)
        self.led_color(self.led_ros, 'red')
        self.led_color(self.led_controller, 'red')
        self.led_color(self.led_battery, 'red')
        self.emergency = True
        self.controller_lister = None
        self.ros_master = None
        self.topic_diag = None
        self.topic_solver = None
        self.controller_running = False
        self.motors = {}
        # remove the LEDS (widgets)
        for k in self.led_motors.keys():
            self.layout_motors.removeWidget(self.led_motors[k])
            self.led_motors[k].deleteLater()
        self.led_motors = {}
        for k in self.led_controllers.keys():
            self.verticalLayout.removeWidget(self.led_controllers[k])
            self.led_controllers[k].deleteLater()
        self.led_controllers = {}
        for k in self.led_topics:
            self.led_color(self.led_topics[k], 'red')

    def update_ros_control(self):
        # always check ROS (again)
        self.update_ros_topics()
        if self.ros_ok:
            try:
                if self.controller_lister == None:
                    rospy.wait_for_service(
                        '/controller_manager/list_controllers', timeout=0.05)
                    self.controller_lister = ControllerLister(
                        '/controller_manager')
                self.controller_list = self.controller_lister()
                self.controller_states = {}
                for i in self.led_controllers.keys():
                    self.controller_states[i] = False
                for c in self.controller_list:
                    if not c.name in self.led_controllers.keys():
                        self.led_controllers[c.name] = QtWidgets.QRadioButton(
                            c.name, self.centralwidget)
                        self.led_controllers[c.name].setObjectName(c.name)
                        self.verticalLayout.insertWidget(
                            len(self.verticalLayout)-1, self.led_controllers[c.name])
                    if c.state == 'running':
                        self.controller_states[c.name] = True
                for c in self.led_controllers.keys():
                    if self.controller_states[c]:
                        self.led_color(self.led_controllers[c], GREEN)
                    else:
                        self.led_color(self.led_controllers[c], 'red')
                self.ros_control_ok = True
                self.led_color(self.led_controller, GREEN)

                # solver
                self.controller_running = False
                for c in self.controller_list:
                    if c.name == 'talos_controller' and c.state == 'running' and self.topic_solver == None:
                        self.topic_solver = rospy.Subscriber(
                            "/talos_controller/iwbc_timings", float64_array, self.solver_cb)
                        self.controller_running = True
            except:
                self.ros_control_ok = False
                self.led_color(self.led_controller, 'red')
                return
        else:
            self.led_color(self.led_controller, 'red')

    def update_diagnostics(self):
        # emergency
        if self.emergency:
            self.led_color(self.led_emergency, 'red')
        else:
            self.led_color(self.led_emergency, GREEN)
        # battery
        self.led_battery.setText("[" + str(self.battery_value) + "%] Battery")
        if self.battery_value < 20 or self.ros_ok != True:
            self.led_color(self.led_battery, 'red')
        elif self.battery_value < 60:
            self.led_color(self.led_battery, 'orange')
        else:
            self.led_color(self.led_battery, GREEN)

        print("#motors:", len(self.motors), self.motors.keys(),
              '#led motors', len(self.led_motors))
        if len(self.led_motors) == 0:
            for k in self.motors.keys():
                self.led_motors[k] = QtWidgets.QRadioButton(
                    k.replace('_motor', ''), self.centralwidget)
                self.led_motors[k].setObjectName(k)
                self.layout_motors.insertWidget(
                    len(self.layout_motors) - 1, self.led_motors[k])
        for k in self.led_motors:
            if k in self.motors.keys():
                if self.motors[k] == 0:
                    self.led_color(self.led_motors[k], GREEN)
                elif self.motors[k] == 1:
                    self.led_color(self.led_motors[k], 'black')
                else:
                    self.led_color(self.led_motors[k], 'red')
            else:
                self.led_color(self.led_motors[k], 'red')

    def led_color(self, b, color):
        b.setStyleSheet(
            "QRadioButton::indicator {width: 14px; height: 14px; border-radius: 7px;} QRadioButton::indicator:unchecked { background-color:" + color + "}")

    # this is the callback used by ROS, not by PyQT

    def diag_cb(self, msg):
        print('diag cb')
        for m in msg.status:
            if m.name == '/Hardware/Battery':
                self.battery_value = float(m.values[0].value.replace("%", ''))
            elif m.name == "/Hardware/Control PC/Load Average":
                self.cpu__.append(float(m.values[1].value))
            elif m.name == "/Hardware/Control PC/Emergency Button":
                if self.robot == 'Tiago':  # Tiago cannot boot with emergency activated
                    self.emergency = False  # => if we are here, the emergency is OK
                else:  # Talos has emergency messages
                    if (m.message == ''):
                        self.emergency = False
                    else:
                        self.emergency = True
            elif "/Hardware/Motor/" in m.name:
                if self.robot == "Talos":
                    n = m.name.split("/")[-1]
                    mode = m.values[8].value
                    if m.message[0] == ' ':
                        self.motors[n] = 0  # OK
                    else:
                        self.motors[n] = 2  # error
                else:  # Tiago
                    n = m.name.split("/")[-1]
                    print(n)
                    for i in m.values:
                        if i.key == "Errors Detected":
                            if i.value == "None":
                                self.motors[n] = 0
                            else:
                                self.motors[n] = 2

    # ROS callback for the solver (not GUI callback)
    def solver_cb(self, msg):
        # print(msg)
        self.solver_queue.append(self.data[0])

    def update_cpu(self):
        pass
        # if not self.robot_ok:
        #     self.plot_cpu.error(True)
        # else:
        #     self.plot_cpu.error(False)
        #     if (len(self.cpu_queue) != 0):
        #         self.plot_cpu.set_data(self.cpu_queue)
        # self.plot_cpu.canvas.ax.set_ylim((0, 10))

    def update_solver(self):
        if not self.controller_running:
            self.plot_solver.error(True)
        else:
            self.plot_solver.error(False)
        if len(self.solver_queue) != 0:
            self.plot_solver.set_data(self.solver_queue)
        # self.plot_solver.canvas.ax.set_ylim((0, 10))

    def __init__(self, conf):
        super(self.__class__, self).__init__()
        self.setupUi(self)
        self.conf = conf

        self.robot = self.conf["robot_name"]
        self.label_robot_name.setText(f"<b>{self.robot}</b>")
        self.button_quit.clicked.connect(QApplication.quit)

        self.battery_value = 0
        self.label_ros_uri.setStyleSheet("font-weight: bold")
        self.led_motors = {}
        # give 100ms to answer
        socket.setdefaulttimeout(float(self.conf['socket_timeout']))

        self.topic_diag = None

        # ping
        # a thread to update data without blocking the GUI
        self.thread_ping = PingThread(self.conf)
        self.thread_ping.start()
        self.thread_ping.new_data.connect(self.plot_widget_ping.new_data)
        self.thread_ping.ok.connect(self.led_robot.set_state)
        self.plot_widget_ping.setYRange(0, self.conf['ping_plot_max'])

        # network stats
        self.timer_network = QTimer()
        self.timer_network.timeout.connect(self.update_network_stats)
        self.timer_network.start(int(self.conf['plot_network_period']))
        self.prev_network_stats = ()
        self.new_data_net_recv_signal.connect(self.plot_downstream.new_data)
        self.new_data_net_sent_signal.connect(self.plot_upstream.new_data)
        

        # ros
        self.ros_ok = False
        self.robot_ok = False
        if USE_ROS:
            self.timer_ros = QTimer()
            self.timer_ros.timeout.connect(self.update_ros_topics)
            self.timer_ros.start(int(self.conf['ros_period']))

            # topic list
            self.topic_list = self.conf['topics']
            self.led_topics = {}
            for k in self.topic_list:
                self.led_topics[k] = QtWidgets.QRadioButton(
                    k, self.centralwidget)
                self.led_topics[k].setObjectName(k)
                self.layout_topics.insertWidget(
                    len(self.layout_topics) - 1, self.led_topics[k])
                self.led_color(self.led_topics[k], 'red')

            self.led_controllers = {}
            self.timer_ros_control = QTimer()
            self.timer_ros_control.timeout.connect(self.update_ros_control)
            self.ros_control_ok = False
            self.timer_ros_control.start(int(self.conf['ros_control_period']))

            # diagnostics (motors, load, etc.)
            self.timer_diag = QTimer()
            self.timer_diag.timeout.connect(self.update_diagnostics)
            self.timer_diag.start(100)  # can be fast because only update GUI

            self.cpu_queue = deque([], maxlen=50)
            self.timer_cpu = QTimer()
            self.timer_cpu.timeout.connect(self.update_cpu)
            self.timer_cpu.start(100)

            self.solver_queue = deque([], maxlen=50)
            self.reinit()
            self.timer_solver = QTimer()
            self.timer_solver.timeout.connect(self.update_solver)
            self.timer_solver.start(100)


def main():

    if not "yaml" in sys.argv[-1]:
        print('usage: {} robot.yaml'.format(sys.argv[0]))
        sys.exit(1)
    conf = yaml.full_load(open(sys.argv[-1]))
    print("loaded: ", sys.argv[-1])

    app = QtWidgets.QApplication(sys.argv)
    dark_style(app)
    screen_size = QDesktopWidget().screenGeometry()

    dashboard = Dashboard(conf)
    dashboard.setGeometry(
        0, 0, int(screen_size.width()/2 * 0.95), screen_size.height())
    dashboard.show()

    video = GstreamerWindow(conf, dashboard)
    video.setGeometry(int(screen_size.width()/2), 0,
                      int(screen_size.width()/2),
                      screen_size.height())
    video.show()
    dashboard.raise_()
    
    video.thread.ready.connect(dashboard.led_gstreamer.set_state)
    dashboard.label_gstreamer_src.setText(conf['gstreamer_launch'].split('!')[0])
    
    app.exec_()




if __name__ == '__main__':
    main()
