import sys
import time
import yaml
import subprocess

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets

from plot_multi import PlotMulti
from led import Led
import plot
import plot_wifi
import wifi

import dark_style

# ROS2import rclpy
import rclpy
import rcl_interfaces
from rclpy.node import Node
from std_msgs.msg import String

from rcl_interfaces.srv import *
from rcl_interfaces.msg import *
import rclpy.parameter

import json
from collections import defaultdict 

# 'packets', 'mean_qbss', 'retries', 'data_packets'
class WifiRos2Thread(QThread):
    bs_signal =  pyqtSignal(dict)
    packets_signal = pyqtSignal(dict)
    mean_qbss_signal = pyqtSignal(dict)
    retries_signal = pyqtSignal(dict)
    data_packets_signal = pyqtSignal(dict)
    
    
    def __init__(self, conf):
        super().__init__()
        rclpy.init()
        self.conf = conf
        self.node = Node('wifimon_client')
        self.wifimon = self.node.create_subscription(String, self.conf['wifi_monitor_topic'], self.monitor_callback, 10)
        print('subscribed')

    def monitor_callback(self, msg):
        d = json.loads(msg.data)
        data = defaultdict(dict) 
        for k,v in d.items():
            data['bs'][int(k)] = v[0]
            data['packets'][int(k)] = v[1]
            data['mean_qbss'][int(k)] = v[2]
            data['retries'][int(k)] = v[3]
            data['data_packets'][int(k)] = v[4]
            
        self.bs_signal.emit(data['bs'])
        self.packets_signal.emit(data['packets'])
        self.mean_qbss_signal.emit(data['mean_qbss'])
        self.retries_signal.emit(data['retries'])
        self.data_packets_signal.emit(data['data_packets'])
        
       
    def run(self):
        while True: # todo simpler spin()
            rclpy.spin_once(self.node)
            time.sleep(0.1)
         
class WifiPlots(QWidget):

    def __init__(self, conf, standalone=True):
        super(QWidget, self).__init__()
        self.conf = conf

        self.layout = QVBoxLayout()
        if standalone:
            self.button_quit = QPushButton('Quit')
            self.layout.addWidget(self.button_quit)
            self.button_quit.clicked.connect(QApplication.quit)
            self.h_layout = QHBoxLayout()
            self.button_save_layout = QPushButton("Save window pos.")
            self.h_layout.addWidget(self.button_save_layout)
            self.button_save_layout_as = QPushButton("as...")
            self.h_layout.addWidget(self.button_save_layout_as)
            self.layout.addLayout(self.h_layout)

        self.led_ros2 = Led('Topic: '  + self.conf['wifi_monitor_topic'])
        self.layout.addWidget(self.led_ros2)


        # wifi
        self.layout_wifi = self.layout

        ## wifi plot
        self.wifi_label = QtWidgets.QLabel('<center><b>WiFi quality</b></center>')
        self.layout_wifi.addWidget(self.wifi_label)
        self.plot_wifi_quality = plot.Plot()
        self.layout_wifi.addWidget(self.plot_wifi_quality)

        ## scanner
        self.plot_wifi = plot_wifi.PlotWifi()
        self.plot_wifi.setMinimumSize(QtCore.QSize(0,400))
        self.layout_wifi.addWidget(self.plot_wifi)

        # WIFI scans
        self.thread_wifi = wifi.WifiScanThread(self.conf)
        self.plot_wifi.setup(self.thread_wifi.channels)
        self.thread_wifi.start()
        self.thread_wifi.networks.connect(self.plot_wifi.new_data)
        self.thread_wifi.essid.connect(lambda s: self.wifi_label.setText(f'<center><b>{s}</b></center>'))
        self.thread_wifi.quality.connect(self.plot_wifi_quality.new_data)
        self.plot_wifi_quality.setYRange(0, 100)
        

        # wifi monitor /sniffing
        plot_names = ['bs', 'packets', 'mean_qbss', 'retries', 'data_packets']
        plot_labels = ['byte/s', 'packets/s', 'mean QBSS', 'retries', 'data packets']
        self.plots = {}
        for p,l in zip(plot_names,plot_labels):
            self.layout.addWidget(QLabel('<center><b>' + l + '</b></center>'))
            self.plots[p] = PlotMulti()
            self.layout.addWidget(self.plots[p])
            self.thread_wifi.channel.connect(self.plots[p].select)

        self.setLayout(self.layout)

        # ros2 thread for monitor
        print('creating ros2 thread...')
        self.thread_ros2 = WifiRos2Thread(self.conf)
        self.thread_ros2.bs_signal.connect(self.plots['bs'].new_data)
        self.thread_ros2.packets_signal.connect(self.plots['packets'].new_data)
        self.thread_ros2.mean_qbss_signal.connect(self.plots['mean_qbss'].new_data)
        self.thread_ros2.retries_signal.connect(self.plots['retries'].new_data)
        self.thread_ros2.data_packets_signal.connect(self.plots['data_packets'].new_data)
        self.thread_ros2.start()
        print('started')

      