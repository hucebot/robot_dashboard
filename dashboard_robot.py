#!/usr/bin/python

from PyQt5.QtWidgets import QApplication, QDesktopWidget, QFileDialog
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
#import gstreamer_plots
#from gstreamer_window import GstreamerWindow
import ping
import led
import plot
#import plot_wifi
import wifi

# for having a control-c
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)


# a nice green for LEDS
GREEN = '#66ff00'

# we can start without ROS (to debug in a train, on Mac, etc.)
USE_ROS = True
try:
    import rospy
    import rostopic
    import rosgraph
    from controller_manager_msgs.msg import ControllerState
    from controller_manager_msgs.srv import *
    from controller_manager_msgs.utils\
        import ControllerLister, ControllerManagerLister,\
        get_rosparam_controller_names
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import WrenchStamped
  #  from std_msgs.msg import Float64MultiArray
    from talos_controller_msgs.msg import float64_array
except Exception as e:
    print("WARNING, ROS 1 disabled: CANNOT IMPORT ROS")
    print(e)
    USE_ROS = False


# this only checks of ros is available (the rest of ROS queries will use timers)
class RosThread(QThread):
    master_online =  pyqtSignal(int)
    ros_control_online =  pyqtSignal(int)
    battery = pyqtSignal(float)
    motors = pyqtSignal(object)
    controllers = pyqtSignal(object)
    load_average =  pyqtSignal(float)
    need_reset = pyqtSignal(bool)
    left_wrist_force = pyqtSignal(float)
    left_wrist_torque = pyqtSignal(float)
    right_wrist_force = pyqtSignal(float)
    right_wrist_torque = pyqtSignal(float)

    def __init__(self, conf):
        super().__init__()
        self.conf = conf
        self.ros_master = None
        self.controller_lister = None
        self.topic_diag = None
        self.controller_states = {}
        self.motor_states = {}
        self.robot = self.conf['robot_name']
        self.done = False
        self.limit_data = 100
        self.left_wrist_force_data = []
        self.left_wrist_torque_data = []
        self.right_wrist_force_data = []
        self.right_wrist_torque_data = []

    def reinit(self):
        self.topic_diag = None
        self.controller_lister = None
        self.ros_master = None
    
    def stop(self):
        self.master_online.emit(0)
        self.ros_control_online.emit(0)
        # if we already called rospy.init_node, we need to restart the whole process
        if self.topic_diag != None: 
            self.need_reset.emit(True)
        else:
            self.controllers.emit({})
            self.motors.emit({})
            self.reinit()

    def check_diagnostics(self):
        if  self.ros_master != None and self.topic_diag == None:
            print("Recreating the diagnostic subscription")
            print("init node/ line 100")
            rospy.init_node('dashboard', anonymous=True,  disable_signals=True)
            self.topic_diag = rospy.Subscriber("/diagnostics", DiagnosticArray, self.diag_cb)
            print("Subscribed to /diagnostics")

    # callback for diagnostics
    def diag_cb(self, msg):
        for m in msg.status:
            if m.name == 'Hardware: Battery':
                self.battery_value = float(m.values[0].value.replace("%", ''))
                self.battery.emit(self.battery_value)
            elif m.name == "Hardware: Control PC: Load Average":
                load_average = float(m.values[1].value)
                self.load_average.emit(load_average)
            elif "Hardware: Motor:" in m.name:
                if self.robot == "Talos":
                    n = m.name.split(":")[-1].replace(" ",'')
                    mode = m.values[8].value
                    if m.message[0] == ' ':
                        self.motor_states[n] = 1  # OK
                    else:
                        self.motor_states[n] = 0  # error
                else:  # Tiago
                    n = m.name.split(":")[-1]
                    for i in m.values:
                        if i.key == "Errors Detected":
                            if i.value == "None":
                                self.motor_states[n] = 1
                            else:
                                self.motor_states[n] = 0
                        # different keys for dynamixels
                        if i.key == "Error Description":
                            if i.value != "":
                                self.motor_states[n] = 0
                            else:
                                self.motor_states[n] = 1
        self.motors.emit(self.motor_states)

    def check_force_and_torque(self):
        if self.ros_master != None:
            rospy.Subscriber(self.conf['left_wrist_topic'], WrenchStamped, self.left_wrist_cb)
            rospy.Subscriber(self.conf['right_wrist_topic'], WrenchStamped, self.right_wrist_cb)

    def left_wrist_cb(self, msg):
        force_module = np.sqrt(msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)
        torque_module = np.sqrt(msg.wrench.torque.x**2 + msg.wrench.torque.y**2 + msg.wrench.torque.z**2)
        self.left_wrist_force_data += [float(force_module)]
        self.left_wrist_torque_data += [float(torque_module)]
        self.left_wrist_force_value = float(msg.wrench.force.x)
        self.left_wrist_torque_value = float(msg.wrench.torque.x)
        if len(self.left_wrist_force_data) > self.limit_data:
            m = np.mean(self.left_wrist_force_data)
            self.left_wrist_force.emit(m)
            self.left_wrist_force_data = []

        if len(self.left_wrist_torque_data) > self.limit_data:
            m = np.mean(self.left_wrist_torque_data)
            self.left_wrist_torque.emit(m)
            self.left_wrist_torque_data = []

    def right_wrist_cb(self, msg):
        force_module = np.sqrt(msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)
        torque_module = np.sqrt(msg.wrench.torque.x**2 + msg.wrench.torque.y**2 + msg.wrench.torque.z**2)
        self.right_wrist_force_data += [float(force_module)]
        self.right_wrist_torque_data += [float(torque_module)]
        self.right_wrist_force_value = float(msg.wrench.force.x)
        self.right_wrist_torque_value = float(msg.wrench.torque.x)
        if len(self.right_wrist_force_data) > self.limit_data:
            m = np.mean(self.right_wrist_force_data)
            self.right_wrist_force.emit(m)
            self.right_wrist_force_data = []

        if len(self.right_wrist_torque_data) > self.limit_data:
            m = np.mean(self.right_wrist_torque_data)
            self.right_wrist_torque.emit(m)
            self.right_wrist_torque_data = []

    def check_ros(self):
        #print("Checking ros...")
        os.environ["ROS_MASTER_URI"] = 'http://' + self.conf['robot_ip'] + ':11311'
        if self.ros_master == None:
            self.ros_master = rosgraph.Master(
                '/rostopic', os.environ["ROS_MASTER_URI"])
        try:
            if self.ros_master.is_online():
                self.master_online.emit(1)
                self.ros_pubs, self.ros_subs = rostopic.get_topic_list(
                    master=self.ros_master)
                #print("ROS Master OK")
            else:
                self.master_online.emit(0)
                self.stop()
               # print("ROS master is not online")
        except Exception as e:
            print("Ros: exception", e)
            self.master_online.emit(0)
            self.stop()

    
    def check_ros_control(self):
         #print("Checking ROS control...")
         if self.ros_master != None:
            try:
                if self.controller_lister == None:
                    rospy.wait_for_service('/controller_manager/list_controllers', timeout=0.05)
                    self.controller_lister = ControllerLister('/controller_manager')
                self.controller_list = self.controller_lister()
                for i in self.controller_states.keys():
                    self.controller_states[i] = False
                for c in self.controller_list:
                    if c.state == 'running':
                        self.controller_states[c.name] = True

                self.ros_control_online.emit(1)
                self.controllers.emit(self.controller_states)
            except Exception as e:
                print("ROS Controller manager not available (exception)", e)
                self.ros_control_online.emit(0)
                self.controller_lister = None



    def run(self):
        while True:
            self.check_ros()
            self.check_ros_control()
            self.check_diagnostics()
            self.check_force_and_torque()
            time.sleep(self.conf['ros_period'] / 1000.0)        

# The new Stream Object which replaces the default stream associated with sys.stdout/stderr
class WriteStream(QObject):
    append_text = pyqtSignal(str)
    set_color = pyqtSignal(QColor)

    def __init__(self, text_edit, color, name):
        super().__init__()
        if not os.path.exists('logs'):
            os.makedirs('logs')
        filename = 'logs/' + name + '_' +  datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".log"
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


class Dashboard(QtWidgets.QMainWindow):
    new_data_net_sent_signal = pyqtSignal(float)
    new_data_net_recv_signal = pyqtSignal(float)

    # TODO move to a thread ?
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
            #sys.exit(2) # we cannot recover from this!
            #self.reinit()

        # diagnostics
        #print("Ros state:", self.led_ros.state, "   topic_diag:", self.topic_diag)
        if self.led_ros.state == 1 and self.topic_diag == None:
            print("Recreating the diagnostic subscription")
            rospy.init_node('dashboard', anonymous=True)
            print("CALLED init_node (ROS1)")
            self.topic_diag = rospy.Subscriber(
                "/diagnostics_agg", DiagnosticArray, self.diag_cb)
            print("subscribed to /diagnostic_agg")

    def reinit(self):
        frame = inspect.stack()[1]
        print("reinit, called from:",
              frame[3] + " line:", frame.frame.f_lineno)
        self.led_ros.set_state(0)
        #self.led_battery.set_state(0)
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
            self.controllers_layout.removeWidget(self.led_controllers[k])
            self.led_controllers[k].deleteLater()
        self.led_controllers = {}
        #for k in self.led_topics:
        #    self.led_color(self.led_topics[k], 'red')
        self.label_ros_uri.setText("[" + os.environ["ROS_MASTER_URI"] + "]")


    def update_controllers_cb(self, controller_list):
        # create new leds if needed
        for c in controller_list.keys():
            if not c in self.led_controllers.keys():
                self.led_controllers[c] = led.Led(c)
                self.led_controllers[c].setObjectName(c)
                self.controllers_layout.insertWidget(len(self.controllers_layout)-1,  self.led_controllers[c])

        # remove leds if needed
        for n in self.led_controllers.keys():
            if not n in controller_list:
                self.controllers_layout.removeWidget(self.led_controllers[n])
                self.led_controllers[n].deleteLater()
                del self.led_controllers[n]

        # some controllers can be loaded but not running
        for n,s in controller_list.items():
            self.led_controllers[n].set_state(s)


    def update_motors_cb(self, motor_list):
        for m in motor_list:
            if not m in self.led_motors.keys():
                self.led_motors[m] = led.Led(m)
                self.led_motors[m].setObjectName(m)
                self.layout_motors.insertWidget(len(self.layout_motors)-1,  self.led_motors[m])
        for n,s in motor_list.items():
            self.led_motors[n].set_state(s)

    def update_ros_control(self):
        # always check ROS (again)
        #self.update_ros_topics()
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
                        self.controllers_layout.insertWidget(
                            len(self.controllers_layout)-1, self.led_controllers[c.name])
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
        #self.led_battery.setText("[" + str(self.battery_value) + "%] Battery")
        # if self.battery_value < 20 or self.led_ros.state != 1:
        #     self.led_color(self.led_battery, 'red')
        # elif self.battery_value < 60:
        #     self.led_color(self.led_battery, 'orange')
        # else:
        #     self.led_color(self.led_battery, GREEN)

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


    def setup_ui(self):
        # the two main layouts
        self.main_layout =  QtWidgets.QVBoxLayout()
        self.setCentralWidget(QtWidgets.QWidget())
        self.centralWidget().setLayout(self.main_layout)
        self.horizontal_layout = QtWidgets.QHBoxLayout()
        self.main_layout.addLayout(self.horizontal_layout)

        # columns
        n_cols = 5
        self.columns = []
        for i in range(n_cols):
            c = QtWidgets.QVBoxLayout()
            self.horizontal_layout.addLayout(c)
            self.columns += [c]

        # quit        
        self.button_quit = QtWidgets.QPushButton('QUIT')
        self.columns[0].addWidget(self.button_quit)

        # window layout
        self.button_save_windows = QtWidgets.QPushButton('Save layout')
        self.columns[0].addWidget(self.button_save_windows)


        # robot name
        self.label_robot_name = QtWidgets.QLabel()
        self.columns[0].addWidget(self.label_robot_name)
        # ros URI
        self.label_ros_uri = QtWidgets.QLabel()
        self.columns[0].addWidget(self.label_ros_uri)
        # ROS core
        self.led_ros = led.Led("ROS Core")
        self.columns[0].addWidget(self.led_ros)
        # a line
        self.line = QtWidgets.QFrame()
        self.columns[0].addWidget(self.line)

        # conntroller list
        self.controllers_layout = self.columns[0]
        self.led_controller = led.Led("Controller Manager")
        self.led_controller.setObjectName("led_controller")
        self.columns[0].addWidget(self.led_controller)
        spacer = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.columns[0].addItem(spacer)

        # motor list
        self.layout_motors = self.columns[1]
        self.label_motors = QtWidgets.QLabel()
        self.layout_motors.addWidget(self.label_motors)
        self.label_motors.setText('<center><b>Motors</b></center>')
        spacer = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.layout_motors.addItem(spacer)

        # watch topic list
        #self.layout_topics = self.columns[2]
        #self.label_topics = QtWidgets.QLabel()
        #self.layout_topics.addWidget(self.label_topics)
        #self.label_topics.setText('<center><b>topics</b></center>')
        #self.layout_topics.addWidget(self.label_topics)
        #spacer = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        #self.layout_topics.addItem(spacer)

        # robot plots
        self.layout_network = self.columns[3]
        self.led_robot = led.Led('Robot computer')
        self.layout_network.addWidget(self.led_robot)

        self.layout_network.addWidget(QtWidgets.QLabel('<center><b>Ping [ms]</b></center>'))
        self.plot_widget_ping = plot.Plot()
        self.layout_network.addWidget(self.plot_widget_ping)

        self.layout_network.addWidget(QtWidgets.QLabel('<center><b>Battery [%]</b></center>'))
        self.plot_battery = plot.Plot()
        self.layout_network.addWidget(self.plot_battery)

        self.layout_network.addWidget(QtWidgets.QLabel('<center><b>Load average</b></center>'))
        self.plot_cpu = plot.Plot()
        self.layout_network.addWidget(self.plot_cpu)


        self.layout_network.addWidget(QtWidgets.QLabel('<center><b>[t -> robot] [Mbps]</b></center>'))
        self.plot_upstream = plot.Plot()
        self.layout_network.addWidget(self.plot_upstream)

        self.layout_network.addWidget(QtWidgets.QLabel('<center><b>[robot -> t] [Mbps]</b></center>'))
        self.plot_downstream = plot.Plot()
        self.layout_network.addWidget(self.plot_downstream)
    
        # Wrist force and torque
        self.layout_network = self.columns[4]

        self.layout_network.addWidget(QtWidgets.QLabel('<center><b> Left Wrist - Force [N]</b></center>'))
        self.left_wrist_force_plot = plot.Plot()
        self.layout_network.addWidget(self.left_wrist_force_plot)

        self.layout_network.addWidget(QtWidgets.QLabel('<center><b> Left Wrist - Torque [Nm]</b></center>'))
        self.left_wrist_torque_plot = plot.Plot()
        self.layout_network.addWidget(self.left_wrist_torque_plot)

        self.layout_network.addWidget(QtWidgets.QLabel('<center><b> Right Wrist - Force [N]</b></center>'))
        self.right_wrist_force_plot = plot.Plot()
        self.layout_network.addWidget(self.right_wrist_force_plot)

        self.layout_network.addWidget(QtWidgets.QLabel('<center><b> Right Wrist - Torque [Nm]</b></center>'))
        self.right_wrist_torque_plot = plot.Plot()
        self.layout_network.addWidget(self.right_wrist_torque_plot)

        # console with output
        self.text_stdout = QtWidgets.QTextEdit()
        self.text_stdout.setObjectName("text_stdout")
        self.main_layout.addWidget(self.text_stdout)




        # # wifi
        # self.layout_wifi = self.columns[4]

        # ## wifi plot
        # self.wifi_label = QtWidgets.QLabel('<center><b>WiFi quality</b></center>')
        # self.layout_wifi.addWidget(self.wifi_label)
        # self.plot_wifi_quality = plot.Plot()
        # self.layout_wifi.addWidget(self.plot_wifi_quality)

        # ## scanner
        # self.plot_wifi = plot_wifi.PlotWifi()
        # self.layout_wifi.addWidget(self.plot_wifi)
       

    def __init__(self, conf):
        super(self.__class__, self).__init__()
        self.setup_ui()
        self.conf = conf

        # connect stdout & stderr
        self.stdout = WriteStream(self.text_stdout, QColor(0, 255, 0), 'stdout')
        self.stderr = WriteStream(self.text_stdout, QColor(255, 0, 0), 'stderr')

        self.robot = self.conf["robot_name"]
        self.label_robot_name.setText(f"<b>{self.robot}</b>")
        self.button_quit.clicked.connect(QApplication.quit)

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
        self.thread_ping.need_reset_signal.connect(sys.exit)
       
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
        
        # WIFI scans
        # self.thread_wifi = wifi.WifiScanThread(self.conf)
        # self.plot_wifi.setup(self.thread_wifi.channels)
        # self.thread_wifi.start()
        # self.thread_wifi.networks.connect(self.plot_wifi.new_data)
        # self.thread_wifi.essid.connect(lambda s: self.wifi_label.setText(f'<center><b>{s}</b></center>'))
        # self.thread_wifi.quality.connect(self.plot_wifi_quality.new_data)
        # self.plot_wifi_quality.setYRange(0, 100)

        # ros
        if USE_ROS:
            # check if ROS is alive
            self.thread_ros = RosThread(self.conf)
            self.thread_ros.start()
            self.thread_ros.master_online.connect(self.led_ros.set_state)
            self.thread_ros.controllers.connect(self.update_controllers_cb)
            self.thread_ros.motors.connect(self.update_motors_cb)
            self.thread_ros.ros_control_online.connect(self.led_controller.set_state)
            self.thread_ros.need_reset.connect(lambda x: sys.exit(2))

            self.thread_ros.left_wrist_force.connect(self.left_wrist_force_plot.new_data)
            self.left_wrist_force_plot.setYRange(-80, 80)
            self.thread_ros.left_wrist_torque.connect(self.left_wrist_torque_plot.new_data)
            self.left_wrist_torque_plot.setYRange(-20, 20)
            self.thread_ros.right_wrist_force.connect(self.right_wrist_force_plot.new_data)
            self.right_wrist_force_plot.setYRange(-80, 80)
            self.thread_ros.right_wrist_torque.connect(self.right_wrist_torque_plot.new_data)
            self.right_wrist_torque_plot.setYRange(-20, 20)

            self.thread_ros.battery.connect(self.plot_battery.new_data)
            self.plot_battery.setYRange(0, 100.01)

            self.thread_ros.load_average.connect(self.plot_cpu.new_data)
            self.plot_cpu.setYRange(0, 10)            

            # topic list            
            #self.topic_list = self.conf['topics']
            #self.led_topics = {}
            #for k in self.topic_list:
            #    self.led_topics[k] = led.Led(k)
            #    self.led_topics[k].setObjectName(k)
            #    self.layout_topics.insertWidget(len(self.layout_topics) - 1, self.led_topics[k])

            self.led_controllers = {}
           
            self.reinit()

def main():

    if not "yaml" in sys.argv[-1]:
        print('usage: {} robot.yaml layout.yaml'.format(sys.argv[0]))
        sys.exit(1)

    conf = yaml.full_load(open(sys.argv[-2]))
    layout = yaml.full_load(open(sys.argv[-1]))
    
    app = QtWidgets.QApplication(sys.argv)
    dark_style.dark_style(app)
    screen_size = QDesktopWidget().screenGeometry()

    dock = 80
    dashboard = Dashboard(conf)
    dashboard.setGeometry(layout['dashboard']['x'], layout['dashboard']['y'], 
                          layout['dashboard']['width'], layout['dashboard']['height'])
    if conf['window_border'] == False:
        dashboard.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
    dashboard.show()


    # gplots = gstreamer_plots.GstreamerPlots(conf, standalone=False) # 80 is because of the dock...
    # gplots.setGeometry(layout['gstreamer_plots']['x'], layout['gstreamer_plots']['y'], 
    #                    layout['gstreamer_plots']['width'], layout['gstreamer_plots']['height'])
    # if conf['window_border'] == False:
    #     gplots.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)

    # gplots.show()

    # video = GstreamerWindow(conf, gplots)
    # video.setGeometry(layout['gstreamer_video']['x'], layout['gstreamer_video']['y'], 
    #                         layout['gstreamer_video']['width'], layout['gstreamer_video']['height'])
    # if conf['window_border'] == False:
    #     video.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
    # video.show()
  
 
    dashboard.raise_()
    
    dashboard.setWindowTitle("Robot: <" + conf['robot_ip'] + ">")
  #  video.setWindowTitle("Video from " + conf['gstreamer_ip'])
   # gplots.setWindowTitle("Gstreamer from "+ conf['gstreamer_ip'])



    # window saving
    def window_size_pos(w):
        d = {}
        d['width'] = w.size().width()
        d['height'] = w.size().height()
        d['x'] = w.pos().x()
        d['y'] = w.pos().y()
        return d

    def save_windows():
        file_name, _ = QFileDialog.getSaveFileName(dashboard, "Save window layout","","All Files (*);;Text Files (*.txt)")
        if file_name:
            with open(file_name, 'w') as file:
                 d = {}
                 d['dashboard'] = window_size_pos(dashboard)
    #             d['gstreamer_plots'] = window_size_pos(gplots)
     #            d['gstreamer_video'] = window_size_pos(video)
                 yaml.dump(d, file)
            print("SAVING WINDOWS:", file_name)
    dashboard.button_save_windows.clicked.connect(save_windows)
  
    app.exec_()




if __name__ == '__main__':
    main()
