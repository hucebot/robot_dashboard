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
        RobotDashBoard.resize(1055, 1040)
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
        self.button_quit = QtWidgets.QPushButton(self.centralwidget)
        self.button_quit.setObjectName("button_quit")
        self.verticalLayout.addWidget(self.button_quit)
        self.label_robot_name = QtWidgets.QLabel(self.centralwidget)
        self.label_robot_name.setObjectName("label_robot_name")
        self.verticalLayout.addWidget(self.label_robot_name)
        self.label_ros_uri = QtWidgets.QLabel(self.centralwidget)
        self.label_ros_uri.setObjectName("label_ros_uri")
        self.verticalLayout.addWidget(self.label_ros_uri)
        self.led_emergency = Led(self.centralwidget)
        self.led_emergency.setObjectName("led_emergency")
        self.verticalLayout.addWidget(self.led_emergency)
        self.led_ros = Led(self.centralwidget)
        self.led_ros.setObjectName("led_ros")
        self.verticalLayout.addWidget(self.led_ros)
        self.led_battery = Led(self.centralwidget)
        self.led_battery.setObjectName("led_battery")
        self.verticalLayout.addWidget(self.led_battery)
        self.line = QtWidgets.QFrame(self.centralwidget)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout.addWidget(self.line)
        self.led_controller = Led(self.centralwidget)
        self.led_controller.setObjectName("led_controller")
        self.verticalLayout.addWidget(self.led_controller)
        spacerItem = QtWidgets.QSpacerItem(20, 10, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout.addItem(spacerItem)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem1)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.layout_motors = QtWidgets.QVBoxLayout()
        self.layout_motors.setObjectName("layout_motors")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setObjectName("label_4")
        self.layout_motors.addWidget(self.label_4)
        spacerItem2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.layout_motors.addItem(spacerItem2)
        self.horizontalLayout.addLayout(self.layout_motors)
        self.layout_topics = QtWidgets.QVBoxLayout()
        self.layout_topics.setObjectName("layout_topics")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setObjectName("label_5")
        self.layout_topics.addWidget(self.label_5)
        spacerItem3 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.layout_topics.addItem(spacerItem3)
        self.horizontalLayout.addLayout(self.layout_topics)
        self.layout_network = QtWidgets.QVBoxLayout()
        self.layout_network.setObjectName("layout_network")
        self.led_robot = Led(self.centralwidget)
        self.led_robot.setObjectName("led_robot")
        self.layout_network.addWidget(self.led_robot)
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setTextFormat(QtCore.Qt.RichText)
        self.label.setObjectName("label")
        self.layout_network.addWidget(self.label)
        self.plot_widget_ping = Plot(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.plot_widget_ping.sizePolicy().hasHeightForWidth())
        self.plot_widget_ping.setSizePolicy(sizePolicy)
        self.plot_widget_ping.setMinimumSize(QtCore.QSize(0, 150))
        self.plot_widget_ping.setMaximumSize(QtCore.QSize(16777215, 150))
        self.plot_widget_ping.setObjectName("plot_widget_ping")
        self.layout_network.addWidget(self.plot_widget_ping)
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setObjectName("label_3")
        self.layout_network.addWidget(self.label_3)
        self.plot_upstream = Plot(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.plot_upstream.sizePolicy().hasHeightForWidth())
        self.plot_upstream.setSizePolicy(sizePolicy)
        self.plot_upstream.setMinimumSize(QtCore.QSize(0, 150))
        self.plot_upstream.setMaximumSize(QtCore.QSize(16777215, 150))
        self.plot_upstream.setObjectName("plot_upstream")
        self.layout_network.addWidget(self.plot_upstream)
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setObjectName("label_7")
        self.layout_network.addWidget(self.label_7)
        self.plot_downstream = Plot(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.plot_downstream.sizePolicy().hasHeightForWidth())
        self.plot_downstream.setSizePolicy(sizePolicy)
        self.plot_downstream.setMinimumSize(QtCore.QSize(0, 150))
        self.plot_downstream.setObjectName("plot_downstream")
        self.layout_network.addWidget(self.plot_downstream)
        self.text_stdout = QtWidgets.QTextEdit(self.centralwidget)
        self.text_stdout.setObjectName("text_stdout")
        self.layout_network.addWidget(self.text_stdout)
        spacerItem4 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.layout_network.addItem(spacerItem4)
        self.horizontalLayout.addLayout(self.layout_network)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.plot_delay = Plot(self.centralwidget)
        self.plot_delay.setObjectName("plot_delay")
        self.verticalLayout_2.addWidget(self.plot_delay)
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
        self.button_quit.setText(_translate("RobotDashBoard", "QUIT"))
        self.label_robot_name.setText(_translate("RobotDashBoard", "Robot-name"))
        self.label_ros_uri.setText(_translate("RobotDashBoard", "http://localhost:11311"))
        self.led_emergency.setText(_translate("RobotDashBoard", "Emergency Stop"))
        self.led_ros.setText(_translate("RobotDashBoard", "ROS"))
        self.led_battery.setText(_translate("RobotDashBoard", "Battery"))
        self.led_controller.setText(_translate("RobotDashBoard", "Controller Manager"))
        self.label_4.setText(_translate("RobotDashBoard", "<b>Motors</b>"))
        self.label_5.setText(_translate("RobotDashBoard", "<b>Topics</b>"))
        self.led_robot.setText(_translate("RobotDashBoard", "Robot"))
        self.label.setText(_translate("RobotDashBoard", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Ping [ms]</span></p></body></html>"))
        self.label_3.setText(_translate("RobotDashBoard", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Upstream (local → robot) [Mb/s]</span></p></body></html>"))
        self.label_7.setText(_translate("RobotDashBoard", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Downstream (robot → local) [Mb/s]</span></p></body></html>"))
from led import Led
from plot import Plot
