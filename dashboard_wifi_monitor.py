#!/usr/bin/python
from PyQt5.QtWidgets import QApplication, QDesktopWidget, QFileDialog
from PyQt5 import QtWidgets, Qt
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import dark_style

import sys
import yaml
import wifi_plots
import win_layout

# for having a control-c
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

def main():

    if not "yaml" in sys.argv[-1]:
        print('usage: {} robot.yaml layout.yaml'.format(sys.argv[0]))
        sys.exit(1)

    conf = yaml.full_load(open(sys.argv[-2]))
    layout = yaml.full_load(open(sys.argv[-1]))
    
    app = QApplication(sys.argv)
    dark_style.dark_style(app)
 
    wplots = wifi_plots.WifiPlots(conf, standalone=True)
    wplots.setGeometry(layout['wifi_plots']['x'], layout['wifi_plots']['y'], 
                       layout['wifi_plots']['width'], layout['wifi_plots']['height'])
    if conf['window_border'] == False:
        wplots.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)

    wplots.show()

    def save_windows_pos(file_name):
        file_name = win_layout.save_window_pos(wplots, 'wifi_plots', file_name)
        
    wplots.button_save_layout.clicked.connect(lambda : save_windows_pos(sys.argv[-1]))
    wplots.button_save_layout_as.clicked.connect(lambda : save_windows_pos(''))
  
    app.exec_()




if __name__ == '__main__':
    main()
