from PyQt5.QtCore import *
import pyqtgraph as pg
import numpy as np
from collections import deque

GREEN = '#66ff00'

class Plot(pg.PlotWidget):

    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.showGrid(x=True, y=True)
        self.line_bg = self.plot([], [], pen=pg.mkPen(color=(102, 255, 0, 128), width=8))
        self.line = self.plot([], [], pen=pg.mkPen(color=(102, 255, 0)))
        self.error = False
        self.data = deque([], maxlen=50) # todo make this 50 configuratble
        self.disableAutoRange()
        self.setXRange(0, 50)
        #self.getPlotItem().hideAxis('bottom')
        self.getPlotItem().showAxis('top')
        self.getPlotItem().showAxis('right')
        self.getPlotItem().getAxis('top').setTicks([])
        self.getPlotItem().getAxis('right').setTicks([])
        self.getPlotItem().getAxis('bottom').setTicks([])

    @pyqtSlot(float)
    def new_data(self, v):
        self.data.append(v)
        self.set_data(self.data)

    def set_data(self, data):
        self.data = data
        self.line_bg.setData(np.arange(len(data)), data)
        self.line.setData(np.arange(len(data)), data)
        #self.setYRange(0, np.max(data))
