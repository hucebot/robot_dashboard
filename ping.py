
import sys
import os
import subprocess
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class PingThread(QThread):
    def update_ping(self):
            ip = self.conf['ip']
            try:
                t = "-t" + str(self.conf['ping_timeout'])
                c = subprocess.check_output(
                    ["fping", "-c1", t, ip], stderr=subprocess.STDOUT)
                t = c.split(b" ")[5]
                p = float(t)
                self.ping_queue.append(p)
                self.led_color(self.led_robot, GREEN)
                self.plot_ping.error(False)
                self.robot_ok = True
            except Exception as e:
                print(e)
                self.led_color(self.led_robot, 'red')
                self.plot_ping.error(True)
                self.robot_ok = False
                self.ping_queue.append(100)  # put a big value to have it on plot
            self.plot_ping.set_data(self.ping_queue)
            self.plot_ping.canvas.ax.set_ylim((0, 30))