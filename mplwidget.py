# Imports
from PyQt5 import QtWidgets
import matplotlib.pyplot
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as Canvas
import matplotlib
import matplotlib.animation
import numpy as np
import copy

# https://stackoverflow.com/questions/36665850/matplotlib-animation-inside-your-own-gui
# Ensure using PyQt5 backend
matplotlib.use('QT5Agg')

# Matplotlib canvas class to create figure
class MplCanvas(Canvas):
    def __init__(self):
        matplotlib.pyplot.style.use('dark_background')
        self.fig = Figure()
        Canvas.__init__(self, self.fig)
        self.ax = self.fig.add_subplot(111)
        self.data = []
        self.error = False

    def set_data(self, y):
        self.data = copy.deepcopy(y)
        self.ax.clear()
        x = np.arange(0, len(self.data))
        if not self.error :
            line, = self.ax.plot(x, self.data, color="#66ff00")
        else:
            line, = self.ax.plot(x, self.data, color="red")

        for cont in range(8, 1, -1):
            self.ax.plot(x, self.data, lw=cont, color=line.get_color(), zorder=5, alpha=0.1)
        self.ax.fill_between(x, y1=[0] * len(self.data), y2=self.data, color=line.get_color(), alpha=0.1)
        #self.ax.set_ylim((0, 30))
        self.ax.set_xlim((0, len(self.data)))
        
        self.ax.grid(color='#111111', ls='--')
        self.draw_idle()

# Matplotlib widget
class MplWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.canvas = MplCanvas()
        self.vbl = QtWidgets.QVBoxLayout()
        self.vbl.addWidget(self.canvas)
        self.setLayout(self.vbl)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        

    def set_data(self, y):
        self.canvas.set_data(y)
        self.canvas.update()
   
    def error(self, v):
        self.canvas.error = v