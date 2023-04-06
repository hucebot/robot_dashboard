from PyQt5.QtCore import pyqtSignal
import pyqtgraph as pg
import numpy as np

GREEN = '#66ff00'

class Plot(pg.PlotWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.showGrid(x=True, y=True)
        self.line_bg = self.plot([], [], pen=pg.mkPen(color=(102, 255, 0, 128), width=8))
        self.line = self.plot([], [], pen=pg.mkPen(color=(102, 255, 0)))
        self.error = False
        # if min != None:
        #     assert(max != None)
        #     self.widget.setYRange(min, max)

    def set_data(self, data):
        self.line_bg.setData(np.arange(len(data)), data)
        self.line.setData(np.arange(len(data)), data)