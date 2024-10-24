
#!/usr/bin/python
# needs to be BEFORE QT
from gstreamer_window import GstreamerWindow
# now we can import QT
from PyQt5.QtWidgets import QApplication, QDesktopWidget, QFileDialog
from PyQt5 import QtWidgets, Qt
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import dark_style

import sys
from collections import deque
import numpy as np
import yaml
import pyqtgraph as pg
import gstreamer_plots
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
    screen_size = QDesktopWidget().screenGeometry()

 
    print('creating gplots...')
    gplots = gstreamer_plots.GstreamerPlots(conf, standalone=True)
    print('gplots created')
    gplots.setGeometry(layout['gstreamer_plots']['x'], layout['gstreamer_plots']['y'], 
                       layout['gstreamer_plots']['width'], layout['gstreamer_plots']['height'])
    if conf['window_border'] == False:
        gplots.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)

    gplots.show()

    list_video = []
    for camera in conf['gstreamer']:
        port = conf['gstreamer'][camera]['port']
        name = conf['gstreamer'][camera]['name']
        layout_x = layout['gstreamer_video'][camera]['x']
        layout_y = layout['gstreamer_video'][camera]['y']
        layout_width = layout['gstreamer_video'][camera]['width']
        layout_height = layout['gstreamer_video'][camera]['height']
        video = GstreamerWindow(conf, gplots, camera)
        video.setGeometry(layout_x, layout_y, layout_width, layout_height)
        if conf['window_border'] == False:
            video.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        video.setWindowTitle("Video from " + conf['gstreamer_ip'] + ' ' + name)
        list_video.append(video)

    for video in list_video:
        video.show()
  
    gplots.setWindowTitle("Gstreamer from "+ conf['gstreamer_ip'])


    def save_windows_pos(file_name):
        file_name = win_layout.save_window_pos(video, 'gstreamer_video', file_name)
        win_layout.save_window_pos(gplots, 'gstreamer_plots', file_name)
        
    gplots.button_save_layout.clicked.connect(lambda : save_windows_pos(sys.argv[-1]))
    gplots.button_save_layout_as.clicked.connect(lambda : save_windows_pos(''))

  
    app.exec_()




if __name__ == '__main__':
    main()
