#!/usr/bin/python

from PyQt5.QtWidgets import QApplication, QDesktopWidget, QFileDialog
from PyQt5 import QtWidgets, Qt
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import dark_style
import sys
import datetime

import yaml
import led
import plot

#ROS2
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String


# for having a control-c
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)


class Thing():
    def __init__(self, topic, conf_iot, ros2_node):
        self.topic = topic
        self.conf_iot = conf_iot
        self.widgets = []
        self.ros2_node = ros2_node
        self.last_msg = -1

        node = topic.split('/')[1].replace('/','')
        if node in self.conf_iot.keys():
            print(self.conf_iot[node])
            widgets = [led.Led(self.conf_iot[node]["name"] + " [" + topic + "]")]
            type =  topic.split('/')[2]
            if type == 'rfid':
                widgets += self.make_rfid(topic)
            elif type == 'tofm2' or type == 'tofm4':
                widgets += self.make_tof(topic)
            elif type == 'key':
                widgets += self.make_key(topic)
            self.widgets = widgets
        else: # simple led if we do not know it
            # should we filter for eurobin_iot?
            self.widgets = [led.Led(topic)]

        self.timer_up = QtCore.QTimer()
        self.timer_up.timeout.connect(self.update_up)
        self.timer_up.start(1000)


    def update_up(self):
        ts = datetime.datetime.today().timestamp()
        if ts - self.last_msg > self.conf_iot['timeout']:
            self.widgets[0].set_state(0)

    def make_tof(self, topic):
        self.subscriber = self.ros2_node.create_subscription(Int16MultiArray, topic, self.listener_tof, 10)
        p = plot.Plot()
        p.setYRange(0, 200)
        return [p]
    
    def make_key(self, topic):
        self.subscriber = self.ros2_node.create_subscription(Int32, topic, self.listener_key, 10)
        p = plot.Plot()
        p.setYRange(0, 1.1)
        return [p]

    def make_rfid(self, topic):
        item_list_w = QtWidgets.QLabel("[No object]")
        self.rfid_item_dict = {}
        # subscribe
        self.subscriber = self.ros2_node.create_subscription(String, topic, self.listener_rfid, 10)
        return [item_list_w]
    
    def listener_rfid(self, msg):
        self.last_msg = datetime.datetime.today().timestamp()
        if len(self.widgets) != 0: # maybe not created yet
            self.widgets[0].set_state(1)
        ts = datetime.datetime.today().timestamp()     
        if msg.data != 'Nothing':
            self.rfid_item_dict[msg.data] = ts
        # only display recent objects
        s = ""
        for key, value  in self.rfid_item_dict.items():
            if ts - value < self.conf_iot['rfid']['timeout']:
                s += f"{self.conf_iot['rfid'][key]} [{(ts-value):.1f}s] \n"
        self.widgets[1].setText(s)

    def listener_tof(self, msg):
        self.last_msg = datetime.datetime.today().timestamp()    
        if len(self.widgets) != 0: # maybe not created yet
            self.widgets[0].set_state(1)
        self.widgets[1].new_data(msg.data[0])
        if msg.data[1] > 500:
            self.widgets[1].set_state(1)
        else:
            self.widgets[1].set_state(0)
    
    def listener_key(self, msg):
        self.last_msg = datetime.datetime.today().timestamp()
        if len(self.widgets) != 0: # maybe not created yet
            self.widgets[0].set_state(1)
        self.widgets[1].new_data(msg.data)


class DashboardIOT(QtWidgets.QMainWindow):
    def __init__(self, conf, conf_iot):
        super(self.__class__, self).__init__()
        self.setup_ui()
        self.conf = conf
        self.conf_iot = conf_iot

        #  ROS2 setup
        rclpy.init()
        self.node = Node("eurobin_iot_dashboard")
        self.ros2_timer = QtCore.QTimer()
        self.ros2_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.1))
        self.ros2_timer.start(100)

        # topic list
        self.update_topic_list()

        self.topic_timer =  QtCore.QTimer()
        self.topic_timer.timeout.connect(self.update_topic_list)
        self.topic_timer.start(100)

    def setup_ui(self):
        # the two main layouts
        self.main_layout =  QtWidgets.QVBoxLayout()
        self.setCentralWidget(QtWidgets.QWidget())
        self.centralWidget().setLayout(self.main_layout)
        self.horizontal_layout = QtWidgets.QHBoxLayout()
        self.main_layout.addLayout(self.horizontal_layout)

        # columns
        n_cols = 1
        self.columns = []
        for i in range(n_cols):
            c = QtWidgets.QVBoxLayout()
            self.horizontal_layout.addLayout(c)
            self.columns += [c]

        # quit        
        self.button_quit = QtWidgets.QPushButton('QUIT')
        self.columns[0].addWidget(self.button_quit)
        self.button_quit.clicked.connect(QApplication.quit)

        # window layout
        self.button_save_windows = QtWidgets.QPushButton('Save layout')
        self.columns[0].addWidget(self.button_save_windows)

        # ROS2 core
        # self.led_ros = led.Led("ROS Core")
        # self.columns[0].addWidget(self.led_ros)

        # a line
        self.line = QtWidgets.QFrame()
        self.columns[0].addWidget(self.line)

        self.topic_list = []
        self.topics = {}

    def update_topic_list(self):
        prev_topic_list = self.topic_list
        self.topic_list = [w[0] for w in self.node.get_topic_names_and_types()]
        if len(prev_topic_list) == len(self.topic_list):
            return
        # widget
        ## new topics
        for topic in self.topic_list:
            if not topic in list(self.topics.keys()):
                self.topics[topic] = Thing(topic, self.conf_iot, self.node)
                if not 'button_a' in topic: 
                    for w in self.topics[topic].widgets:
                        w.setObjectName(topic)
                        self.columns[0].insertWidget(len(self.columns[0]) - 1, w)

        ## removed topics
        wlist = list(self.topics.keys())
        for w in wlist:
            if not w in self.topics:
                for widget in w.widgets:
                    self.columns[0].removeWidget(widget)
                    widget.deleteLater()
                    del widget
                # todo also delete the Device object
                print("deleted topic:", w)
        print("OK")
        


    


def main():
    if not "yaml" in sys.argv[-1]:
        print('usage: {} robot.yaml layout.yaml iot.yaml'.format(sys.argv[0]))
        sys.exit(1)

    conf = yaml.full_load(open(sys.argv[-3]))
    layout = yaml.full_load(open(sys.argv[-2]))
    conf_iot =  yaml.full_load(open(sys.argv[-1]))
    
    app = QtWidgets.QApplication(sys.argv)
    dark_style.dark_style(app)
    screen_size = QDesktopWidget().screenGeometry()


    dashboard = DashboardIOT(conf, conf_iot)
    dashboard.setGeometry(layout['dashboard_iot']['x'], layout['dashboard_iot']['y'], 
                          layout['dashboard_iot']['width'], layout['dashboard_iot']['height'])
    if conf['window_border'] == False:
        dashboard.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
    
    dashboard.show()
    dashboard.raise_()
    dashboard.setWindowTitle("euROBIN IOT")


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
            with open(file_name, 'r') as file:
                d = yaml.safe_load(file) or {}
                d['dashboard_iot'] = window_size_pos(dashboard)
            with open(file_name, 'w') as file:
                yaml.dump(d, file)

            print("SAVING WINDOWS:", file_name)
    dashboard.button_save_windows.clicked.connect(save_windows)
  


    app.exec_()




if __name__ == '__main__':
    main()
