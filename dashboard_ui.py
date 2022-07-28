# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'dashboard.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_RobotDashBoard(object):
    def setupUi(self, RobotDashBoard):
        RobotDashBoard.setObjectName("RobotDashBoard")
        RobotDashBoard.resize(731, 505)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(RobotDashBoard.sizePolicy().hasHeightForWidth())
        RobotDashBoard.setSizePolicy(sizePolicy)
        self.centralwidget = QtWidgets.QWidget(RobotDashBoard)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_ros_uri = QtWidgets.QLabel(self.centralwidget)
        self.label_ros_uri.setObjectName("label_ros_uri")
        self.verticalLayout.addWidget(self.label_ros_uri)
        self.led_robot = QtWidgets.QRadioButton(self.centralwidget)
        self.led_robot.setObjectName("led_robot")
        self.verticalLayout.addWidget(self.led_robot)
        self.led_emergency = QtWidgets.QRadioButton(self.centralwidget)
        self.led_emergency.setObjectName("led_emergency")
        self.verticalLayout.addWidget(self.led_emergency)
        self.led_ros = QtWidgets.QRadioButton(self.centralwidget)
        self.led_ros.setObjectName("led_ros")
        self.verticalLayout.addWidget(self.led_ros)
        self.label_battery = QtWidgets.QLabel(self.centralwidget)
        self.label_battery.setObjectName("label_battery")
        self.verticalLayout.addWidget(self.label_battery)
        self.line = QtWidgets.QFrame(self.centralwidget)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout.addWidget(self.line)
        self.led_controller = QtWidgets.QRadioButton(self.centralwidget)
        self.led_controller.setObjectName("led_controller")
        self.verticalLayout.addWidget(self.led_controller)
        spacerItem = QtWidgets.QSpacerItem(20, 10, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout.addItem(spacerItem)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem1)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.layout_motors = QtWidgets.QVBoxLayout()
        self.layout_motors.setObjectName("layout_motors")
        spacerItem2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.layout_motors.addItem(spacerItem2)
        self.horizontalLayout.addLayout(self.layout_motors)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.plot_ping = MplWidget(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.plot_ping.sizePolicy().hasHeightForWidth())
        self.plot_ping.setSizePolicy(sizePolicy)
        self.plot_ping.setMinimumSize(QtCore.QSize(0, 10))
        self.plot_ping.setObjectName("plot_ping")
        self.verticalLayout_2.addWidget(self.plot_ping)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        RobotDashBoard.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(RobotDashBoard)
        self.statusbar.setObjectName("statusbar")
        RobotDashBoard.setStatusBar(self.statusbar)

        self.retranslateUi(RobotDashBoard)
        QtCore.QMetaObject.connectSlotsByName(RobotDashBoard)

    def retranslateUi(self, RobotDashBoard):
        _translate = QtCore.QCoreApplication.translate
        RobotDashBoard.setWindowTitle(_translate("RobotDashBoard", "MainWindow"))
        self.label_ros_uri.setText(_translate("RobotDashBoard", "http://localhost:11311"))
        self.led_robot.setText(_translate("RobotDashBoard", "Robot"))
        self.led_emergency.setText(_translate("RobotDashBoard", "Emergency Stop"))
        self.led_ros.setText(_translate("RobotDashBoard", "ROS"))
        self.label_battery.setText(_translate("RobotDashBoard", "Battery: [?]"))
        self.led_controller.setText(_translate("RobotDashBoard", "Controller Manager"))
from mplwidget import MplWidget
