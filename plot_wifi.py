from PyQt5.QtCore import *
from pyqtgraph.Qt import QtGui
import pyqtgraph as pg
import numpy as np

from collections import defaultdict

GREEN = '#66ff00'
PLOT_COLORS = ['#e6194b', '#3cb44b', '#ffe119', '#4363d8', '#f58231', '#911eb4', '#46f0f0', '#f032e6', '#bcf60c', '#fabebe', '#008080', '#e6beff', '#9a6324', '#fffac8', '#800000', '#aaffc3', '#808000', '#ffd8b1', '#000075', '#808080']

class PlotWifi(pg.PlotWidget):

    def __init__(self, parent=None):
        super().__init__(parent=parent)

    def setup(self, channel_list):
        # the list of channels supported by the board
        self.channel_list = channel_list 
    
        self.scatter = pg.ScatterPlotItem(pen=None)
        self.showGrid(x=True, y=True, alpha=0.2)
        
        self.disableAutoRange()
        self.setXRange(0, len(channel_list))

        # channels as x
        print('channel list:', channel_list)
        labels = [(i, str(int(channel_list[i]))) for i in range(len(channel_list))]
        self.getPlotItem().getAxis('bottom').setTicks([labels])

        self.channel_to_x =  {int(channel_list[i]):i for i in range(len(channel_list))}

        font = QtGui.QFont()
        font.setPixelSize(10)
        self.getPlotItem().getAxis("bottom").setStyle(tickFont = font)
        font = QtGui.QFont()
        font.setWeight(QtGui.QFont.Bold)
        font.setPixelSize(12)
        self.getPlotItem().getAxis("left").setStyle(tickFont = font)
        self.getPlotItem().showAxis('top')
        self.getPlotItem().showAxis('right')
        self.getPlotItem().getAxis('top').setTicks([])
        self.getPlotItem().getAxis('right').setTicks([])


        
        for i in range(len(channel_list)):
            l = pg.InfiniteLine(pos=i, pen=pg.mkPen(color=pg.intColor(int(channel_list[i]))))
            self.addItem(l)

        self.addItem(self.scatter)
        self.brushes = { i:pg.mkBrush(255 - (i / 100. * 255), (i / 100. * 255), 0, 255) for i in range(101) }


    @pyqtSlot(object)
    def new_data(self, net_list):
        self.networks = net_list

        # each ESSID can have many channels (Mesh)!
        by_essid = defaultdict(list)
        for i in self.networks:
            by_essid[i['Name']] += [i]

        # ESSID has Y
        essid_list = sorted(list(by_essid.keys()))
        labels = [(i, essid_list[i]) for i in range(len(essid_list))]
        self.getPlotItem().getAxis('left').setTicks([labels])
        self.essid_to_y =  { essid_list[i]:i for i in range(len(essid_list)) }
        self.setYRange(0, len(self.essid_to_y))
        #print("networks:", self.networks)
        pos = [(self.channel_to_x[int(n['Channel'])], self.essid_to_y[n['Name']]) for n in self.networks]
        size = [int(n['Quality']) / 3. for n in self.networks]
        brushes = [self.brushes[int(n['Quality'])] for n in self.networks]
        self.scatter.setData(pos=pos, size=size, brush=brushes)

   


    # def setup(self, channel_list):
    #     # the list of channels supported by the board
    #     self.channel_list = channel_list 
    #     # the list of bar indices for each channel
    #     self.channel_ind = {} 
    #     # the y coordinate of each bar
    #     self.y = [] 

    #     # indices counter
    #     k = 0
    #     num_slots = 15
    #     for i in range(len(channel_list)):
    #         ind = []
    #         for j in np.arange(-0.5, 0.5, 1./num_slots): # 20 slots
    #             yy = i * 2 + j + 0.1
    #             self.y += [yy]
    #             ind += [k]
    #             k += 1
    #         self.channel_ind[int(channel_list[i])] = ind
                
    #     self.x = [20] * len(self.y) #[5, 5, 7, 10, 3, 8, 9, 1, 6, 2]
    #     brushes = [PLOT_COLORS[i % len(PLOT_COLORS)] for i in range(len(self.y))]
    #     self.bargraph = pg.BarGraphItem(x0=0, y=self.y, height=0.1, width=self.x, brushes=brushes,pen=None)
    #     self.addItem(self.bargraph)
    #     self.showGrid(x=True, y=False)
    #     self.disableAutoRange()
    #     self.setXRange(0, 100)
    #     self.setYRange(0, len(channel_list) * 2)
    #     #self.getPlotItem().hideAxis('left')
    #     ay = self.getPlotItem().getAxis('left')
    #     labels = [(i*2, channel_list[i]) for i in range(len(channel_list))]
    #     ay.setTicks([labels])
    #     print([range(len(channel_list)), channel_list])

    # @pyqtSlot(object)
    # def new_data(self, n):
    #     self.networks = n
    #     self.by_channel = defaultdict(list)
    #     self.x = [0] * len(self.y)
    #     for i in n:
    #         self.by_channel[int(i['Channel'])] += [i]
    #     print("L:", [len(x) for x in self.by_channel.values()])

    #     for (channel, net_list) in self.by_channel.items():
    #         for k in range(len(net_list)):
    #             self.x[self.channel_ind[channel][k]] = int(net_list[k]['Quality'])
    #             print("Q:", net_list[k]['Quality'])
    #     print(self.x)
    #     self.bargraph.setOpts(width=self.x)

   