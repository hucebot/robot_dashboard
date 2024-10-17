#!/usr/bin/python

import sys
import rospy
from PyQt5.QtWidgets import QApplication, QLabel, QWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from geometry_msgs.msg import WrenchStamped
import numpy as np

class AlertApp(QWidget):
    def __init__(self, conf, layout, topic, side):
        super().__init__()
        self.conf = conf
        self.layout = layout
        self.side = side
        if side == 'left':
            self.setGeometry(self.layout['left_gripper_alarm']['x'], self.layout['left_gripper_alarm']['y'], 800, 200)
            self.font_size = self.layout['left_gripper_alarm']['font_size']
            self.setWindowTitle("Left Gripper Alarm")
            self.label = QLabel(self)
            self.label.setGeometry(self.layout['left_gripper_alarm']['label_x'], self.layout['left_gripper_alarm']['label_y'], 800, 200)

        else:
            self.setGeometry(self.layout['right_gripper_alarm']['x'], self.layout['right_gripper_alarm']['y'], 800, 200)
            self.font_size = self.layout['right_gripper_alarm']['font_size']
            self.setWindowTitle("Right Gripper Alarm")
            self.label = QLabel(self)
            self.label.setGeometry(self.layout['right_gripper_alarm']['label_x'], self.layout['right_gripper_alarm']['label_y'], 800, 200)

        self.setWindowFlag(Qt.WindowStaysOnTopHint)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        
        self.label.setStyleSheet(f"color: red; font-size: {self.font_size}px;")
        self.label.setText("")
        self.label.setFont(QFont("Arial", self.font_size, QFont.Bold))
        self.label.setAlignment(Qt.AlignCenter)

        rospy.Subscriber(topic, WrenchStamped, self.callback)

    def callback(self, msg):
        force_module = np.sqrt(msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)
        torque_module = np.sqrt(msg.wrench.torque.x**2 + msg.wrench.torque.y**2 + msg.wrench.torque.z**2)
        if force_module > (self.conf['limit_force']- 10.0 ) or torque_module > (self.conf['limit_torque'] - 5.0):
            text = f'beware with the {self.side} gripper'.upper()
            self.label.setText(text)
        else:
            self.label.setText("")

    def update_text(self, text):
        self.label.setText(text)