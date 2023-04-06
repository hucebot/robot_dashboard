# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'dashboard.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_RobotDashBoard(object):
    def setupUi(self, RobotDashBoard):
        RobotDashBoard.setObjectName("RobotDashBoard")
        RobotDashBoard.resize(1063, 774)
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
        self.led_robot = Led(self.centralwidget)
        self.led_robot.setObjectName("led_robot")
        self.verticalLayout.addWidget(self.led_robot)
        self.led_emergency = QtWidgets.QRadioButton(self.centralwidget)
        self.led_emergency.setObjectName("led_emergency")
        self.verticalLayout.addWidget(self.led_emergency)
        self.led_ros = QtWidgets.QRadioButton(self.centralwidget)
        self.led_ros.setObjectName("led_ros")
        self.verticalLayout.addWidget(self.led_ros)
        self.led_battery = QtWidgets.QRadioButton(self.centralwidget)
        self.led_battery.setObjectName("led_battery")
        self.verticalLayout.addWidget(self.led_battery)
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
        self.layout_plots = QtWidgets.QVBoxLayout()
        self.layout_plots.setObjectName("layout_plots")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setTextFormat(QtCore.Qt.RichText)
        self.label.setObjectName("label")
        self.layout_plots.addWidget(self.label)
        self.plot_widget_ping = PlotWidget(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.plot_widget_ping.sizePolicy().hasHeightForWidth())
        self.plot_widget_ping.setSizePolicy(sizePolicy)
        self.plot_widget_ping.setMinimumSize(QtCore.QSize(0, 10))
        self.plot_widget_ping.setObjectName("plot_widget_ping")
        self.layout_plots.addWidget(self.plot_widget_ping)
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setObjectName("label_2")
        self.layout_plots.addWidget(self.label_2)
        self.plot_widget_cpu = PlotWidget(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.plot_widget_cpu.sizePolicy().hasHeightForWidth())
        self.plot_widget_cpu.setSizePolicy(sizePolicy)
        self.plot_widget_cpu.setMinimumSize(QtCore.QSize(100, 100))
        self.plot_widget_cpu.setObjectName("plot_widget_cpu")
        self.gridLayout = QtWidgets.QGridLayout(self.plot_widget_cpu)
        self.gridLayout.setObjectName("gridLayout")
        self.layout_plots.addWidget(self.plot_widget_cpu)
        self.horizontalLayout.addLayout(self.layout_plots)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.led_gstreamer = QtWidgets.QRadioButton(self.centralwidget)
        self.led_gstreamer.setObjectName("led_gstreamer")
        self.verticalLayout_2.addWidget(self.led_gstreamer)
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setObjectName("label_3")
        self.verticalLayout_2.addWidget(self.label_3)
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setObjectName("label_6")
        self.verticalLayout_2.addWidget(self.label_6)
        spacerItem4 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem4)
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
        self.led_battery.setText(_translate("RobotDashBoard", "Battery"))
        self.led_controller.setText(_translate("RobotDashBoard", "Controller Manager"))
        self.label_4.setText(_translate("RobotDashBoard", "<b>Motors</b>"))
        self.label_5.setText(_translate("RobotDashBoard", "<b>Topics</b>"))
        self.label.setText(_translate("RobotDashBoard", "<center><b>Ping (ms)</b></center>"))
        self.label_2.setText(_translate("RobotDashBoard", "<center><b>Load average</b></center>"))
        self.led_gstreamer.setText(_translate("RobotDashBoard", "Gstreamer"))
        self.label_3.setText(_translate("RobotDashBoard", "UDP Port:"))
        self.label_6.setText(_translate("RobotDashBoard", "FPS:"))
from led import Led
from pyqtgraph import PlotWidget
