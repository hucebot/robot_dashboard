from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import time
import subprocess

# ping is blocking and we do not want that
class PingThread(QThread):
    new_data = pyqtSignal(float)
    need_reset_signal = pyqtSignal(int)
    ok = pyqtSignal(int)
    ping = 0
    bad_ping_counter = 0
    robot_started = False

    def __init__(self, conf, ip):
        super().__init__()
        self.conf = conf
        self.ip = ip
      #  print("IP:", self.ip, self)

    def run(self):
        while True:
            try:
               # print("self.ip:", self.ip)
                t = "-t" + str(self.conf['ping_timeout'])
                c = subprocess.check_output(
                    ["fping", "-c1", t, self.ip], stderr=subprocess.STDOUT)
                t = c.split(b" ")[5]
                self.ping = float(t)
                self.ok.emit(1)
                self.robot_started = True # we started once
                self.bad_ping_counter = 0
            except Exception as e:
                print(e)
                self.ping = self.conf['ping_timeout']
                self.ok.emit(0)
                self.bad_ping_counter += 1
            #print("emit new data")
            self.new_data.emit(self.ping)
            time.sleep(self.conf['ping_period'])
            #print("bad ping counter:", self.bad_ping_counter, "  started:", self.robot_started)
            if self.robot_started and self.bad_ping_counter >= self.conf['max_bad_ping']:
                print("Ping: need restart (too many bad pings -- robot is down)")
                self.need_reset_signal.emit(2) # exit with 2
            if self.bad_ping_counter != 0:
                self.ok.emit(2)