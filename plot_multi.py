from PyQt5.QtCore import *
import pyqtgraph as pg
import numpy as np
from collections import deque

GREEN = '#66ff00'

class PlotMulti(pg.PlotWidget):

    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.showGrid(x=True, y=True)
        self.lines = {}
        self.labels = {}
        self.data = {}
        self.highlighted = -1
        #self.disableAutoRange()
        self.setXRange(0, 50)
        #self.getPlotItem().hideAxis('bottom')
        self.getPlotItem().showAxis('top')
        self.getPlotItem().showAxis('right')
        self.getPlotItem().getAxis('top').setTicks([])
        self.getPlotItem().getAxis('right').setTicks([])
        self.getPlotItem().getAxis('bottom').setTicks([])
        #self.setBackground((10, 10, 10))

    @pyqtSlot(int)
    def select(self, v):
        if self.highlighted != v:
            if self.highlighted != -1:
                self.lines[self.highlighted].setPen(pg.mkPen(width=1,color=pg.intColor(v)))
            self.lines[v].setPen(pg.mkPen(width=6,color=pg.intColor(v)))
            self.highlighted = v
        
    @pyqtSlot(dict)
    def new_data(self, v):
        # first call
        if not self.lines:
            i = 0
            for k in v.keys():
                self.lines[k] = self.plot([], [], pen=pg.mkPen(color=pg.intColor(k)))
                self.data[k] = deque([], maxlen=50)
                self.labels[k] = pg.TextItem(str(k), color=pg.intColor(k))
                self.addItem(self.labels[k])
                i += 1
        # update the plot
        for k,v in v.items():
            self.data[k].append(v)     
            self.lines[k].setData(np.arange(len(self.data[k])), self.data[k])
            self.labels[k].setPos(-2, self.data[k][0])