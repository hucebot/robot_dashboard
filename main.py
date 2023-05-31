#!/usr/bin/python
from gstreamer_window import GstreamerWindow
from PyQt5.QtWidgets import QStyleFactory
from PyQt5.QtWidgets import QApplication, QDesktopWidget
import dashboard_ui
from PyQt5 import QtWidgets, Qt
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import dark_style
import inspect

import datetime
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
import gstreamer_plots
import ping

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



# The new Stream Object which replaces the default stream associated with sys.stdout/stderr
class WriteStream(QObject):
    append_text = pyqtSignal(str)
    set_color = pyqtSignal(QColor)

    def __init__(self, text_edit, color, name):
        super().__init__()
        filename = name + '_' +  datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".log"
        self.color = color
        self.log_file = open(filename, "w")
        self.text_edit = text_edit
        # could be moved elsewhere
        self.text_edit.setReadOnly(True)
        self.sb = self.text_edit.verticalScrollBar()
        self.sb.setValue(self.sb.maximum())

        self.append_text.connect(self.append)
        self.set_color.connect(self.text_edit.setTextColor)
        self.stderr = sys.stderr
        self.stdout = sys.stdout
        self.name = name
        if name == 'stdout':
            sys.stdout = self
        if name == 'stderr':
            sys.stderr = self

    def append(self, msg):
        self.text_edit.insertPlainText(msg)
        # automatic scrolling
        sb = self.text_edit.verticalScrollBar()
        sb.setValue(sb.maximum())

    def write(self, text):
        self.log_file.write(text)
        if self.name == 'stdout':
            self.stdout.write(text)
        if self.name == 'stderr':
            self.stderr.write(text)

        # we need to use signals/slots to be thread safe
        self.set_color.emit(self.color)
        self.append_text.emit(text)

    def flush(self):
        if self.name == 'stdout':
            self.stdout.flush()
        if self.name == 'stderr':
            self.stderr.flush()
        self.log_file.flush()


class Dashboard(QtWidgets.QMainWindow, dashboard_ui.Ui_RobotDashBoard):
    new_data_net_sent_signal = pyqtSignal(float)
    new_data_net_recv_signal = pyqtSignal(float)

    # TODO move to a thread
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
            print("update_network_stats::Interface not found")

    def update_ros_topics(self):
        #if self.led_robot.state != 1:
        #    self.led_ros.set_state(0)
            #self.reinit()
        #    return
        self.ros_pubs = []
        os.environ["ROS_MASTER_URI"] = 'http://' + self.conf['robot_ip'] + ':11311'
        self.label_ros_uri.setText("[" + os.environ["ROS_MASTER_URI"] + "]")
        if self.ros_master == None:
            self.ros_master = rosgraph.Master(
                '/rostopic', os.environ["ROS_MASTER_URI"])
        try:
            if self.ros_master.is_online():
                self.led_ros.set_state(1)
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
                self.led_ros.set_state(0)
        except Exception as e:
            self.led_ros.set_state(0)
            print("Ros: exception", e)

        if self.led_robot == 1 and self.led_ros.state != 1:
            print("ROS is down, but robot is up, restarting")
            sys.exit(2) # we cannot recover from this!
            #self.reinit()

        # diagnostics
        #print("Ros state:", self.led_ros.state, "   topic_diag:", self.topic_diag)
        if self.led_ros.state == 1 and self.topic_diag == None:
            print("Recreating the diagnostic subscription")
            rospy.init_node('dashboard', anonymous=True)
            self.topic_diag = rospy.Subscriber(
                "/diagnostics_agg", DiagnosticArray, self.diag_cb)

    def reinit(self):
        frame = inspect.stack()[1]
        print("reinit, called from:",
              frame[3] + " line:", frame.frame.f_lineno)
        self.led_ros.set_state(0)
        self.led_battery.set_state(0)
        self.led_controller.set_state(0)
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
        if self.led_ros.state == 1:
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
        if self.battery_value < 20 or self.led_ros.state != 1:
            self.led_color(self.led_battery, 'red')
        elif self.battery_value < 60:
            self.led_color(self.led_battery, 'orange')
        else:
            self.led_color(self.led_battery, GREEN)

        #print("#motors:", len(self.motors), self.motors.keys(),
        #      '#led motors', len(self.led_motors))
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
        #print('diag cb')
        for m in msg.status:
            if m.name == '/Hardware/Battery':
                self.battery_value = float(m.values[0].value.replace("%", ''))
            elif m.name == "/Hardware/Control PC/Load Average":
                cpu = float(m.values[1].value)
                # TODO CPU plot
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
                    for i in m.values:
                        if i.key == "Errors Detected":
                            if i.value == "None":
                                self.motors[n] = 0
                            else:
                                self.motors[n] = 2
                        # different keys for dynamixels
                        if i.key == "Error Description":
                            if i.value != "":
                                self.motors[n] = 2
                            else:
                                self.motors[n] = 0

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
        pass
        # if not self.controller_running:
        #     self.plot_solver.error(True)
        # else:
        #     self.plot_solver.error(False)
        # if len(self.solver_queue) != 0:
        #     self.plot_solver.set_data(self.solver_queue)
        # self.plot_solver.canvas.ax.set_ylim((0, 10))

    def __init__(self, conf):
        super(self.__class__, self).__init__()
        self.setupUi(self)
        self.conf = conf

        # connect stdout & stderr
        self.stdout = WriteStream(self.text_stdout, QColor(0, 255, 0), 'stdout')
        self.stderr = WriteStream(self.text_stdout, QColor(255, 0, 0), 'stderr')

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
        self.thread_ping = ping.PingThread(self.conf, self.conf['robot_ip'])
        self.thread_ping.start()
        self.thread_ping.new_data.connect(self.plot_widget_ping.new_data)
        self.thread_ping.ok.connect(self.led_robot.set_state)
        self.thread_ping.need_reset_signal.connect(lambda x: sys.exit(2))
       
        # ranges
        self.plot_widget_ping.setYRange(0, self.conf['plot_ping_max'])
        self.plot_downstream.setYRange(0, self.conf['plot_downstream_max'])
        self.plot_upstream.setYRange(0, self.conf['plot_upstream_max'])
        
        # network stats
        self.timer_network = QTimer()
        self.timer_network.timeout.connect(self.update_network_stats)
        self.timer_network.start(int(self.conf['plot_network_period']))
        self.prev_network_stats = ()
        self.new_data_net_recv_signal.connect(self.plot_downstream.new_data)
        self.new_data_net_sent_signal.connect(self.plot_upstream.new_data)
        

        # ros
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
        print('usage: {} [--no-stdout-redirect] robot.yaml'.format(sys.argv[0]))
        sys.exit(1)
    conf = yaml.full_load(open(sys.argv[-1]))
    print("loaded: ", sys.argv[-1])

    app = QtWidgets.QApplication(sys.argv)
    dark_style.dark_style(app)
    screen_size = QDesktopWidget().screenGeometry()

    dashboard = Dashboard(conf)
    dashboard.setGeometry(
        0, 0, int(screen_size.width()/4), screen_size.height())
   # dashboard.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
    dashboard.show()

    gplots = gstreamer_plots.GstreamerPlots(conf)
    gplots.setGeometry(int(screen_size.width())/4, 0, int(screen_size.width()/4),
                      screen_size.height())
    gplots.show()

    video = GstreamerWindow(conf, gplots)
    video.setGeometry(int(screen_size.width()/2), 0,
                      int(screen_size.width()/2),
                      screen_size.height())
   # video.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
    video.show()
 
 
 
    dashboard.raise_()
    
    dashboard.setWindowTitle("Robot: <" + conf['robot_ip'] + ">")
    video.setWindowTitle("Video from " + conf['gstreamer_ip'])
    gplots.setWindowTitle("Gstreamer from "+ conf['gstreamer_ip'])
    app.exec_()




if __name__ == '__main__':
    main()
