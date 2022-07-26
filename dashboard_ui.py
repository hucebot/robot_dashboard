# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'dashboard.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_RobotDashBoard(object):
    def setupUi(self, RobotDashBoard):
        RobotDashBoard.setObjectName("RobotDashBoard")
        RobotDashBoard.resize(731, 505)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
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
        self.drop_ip = QtWidgets.QComboBox(self.centralwidget)
        self.drop_ip.setCurrentText("")
        self.drop_ip.setObjectName("drop_ip")
        self.verticalLayout.addWidget(self.drop_ip)
        self.led_robot = QtWidgets.QRadioButton(self.centralwidget)
        self.led_robot.setObjectName("led_robot")
        self.verticalLayout.addWidget(self.led_robot)
        self.led_ros = QtWidgets.QRadioButton(self.centralwidget)
        self.led_ros.setObjectName("led_ros")
        self.verticalLayout.addWidget(self.led_ros)
        self.led_motors = QtWidgets.QRadioButton(self.centralwidget)
        self.led_motors.setObjectName("led_motors")
        self.verticalLayout.addWidget(self.led_motors)
        self.led_controller = QtWidgets.QRadioButton(self.centralwidget)
        self.led_controller.setObjectName("led_controller")
        self.verticalLayout.addWidget(self.led_controller)
        self.led_joystick = QtWidgets.QRadioButton(self.centralwidget)
        self.led_joystick.setObjectName("led_joystick")
        self.verticalLayout.addWidget(self.led_joystick)
        self.led_geomagic = QtWidgets.QRadioButton(self.centralwidget)
        self.led_geomagic.setObjectName("led_geomagic")
        self.verticalLayout.addWidget(self.led_geomagic)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.plot_ping = MplWidget(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.plot_ping.sizePolicy().hasHeightForWidth())
        self.plot_ping.setSizePolicy(sizePolicy)
        self.plot_ping.setMinimumSize(QtCore.QSize(0, 10))
        self.plot_ping.setObjectName("plot_ping")
        self.horizontalLayout.addWidget(self.plot_ping)
        RobotDashBoard.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(RobotDashBoard)
        self.statusbar.setObjectName("statusbar")
        RobotDashBoard.setStatusBar(self.statusbar)

        self.retranslateUi(RobotDashBoard)
        QtCore.QMetaObject.connectSlotsByName(RobotDashBoard)

    def retranslateUi(self, RobotDashBoard):
        _translate = QtCore.QCoreApplication.translate
        RobotDashBoard.setWindowTitle(_translate("RobotDashBoard", "MainWindow"))
        self.led_robot.setText(_translate("RobotDashBoard", "Robot"))
        self.led_ros.setText(_translate("RobotDashBoard", "ROS"))
        self.led_motors.setText(_translate("RobotDashBoard", "Motors"))
        self.led_controller.setText(_translate("RobotDashBoard", "IWBC Controller"))
        self.led_joystick.setText(_translate("RobotDashBoard", "Joystick"))
        self.led_geomagic.setText(_translate("RobotDashBoard", "Geomagic"))
from mplwidget import MplWidget
