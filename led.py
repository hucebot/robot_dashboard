from PyQt5.QtWidgets import QRadioButton
from PyQt5.QtCore import *

GREEN = '#66ff00'

class Led(QRadioButton):

    repaint_signal = pyqtSignal()

    def __init__(self, text):
        super().__init__(text)
        self.state = 0
        self.setDisabled(True)
        self.set_state(self.state)

    @pyqtSlot(int)
    def set_state(self, state):
        self.state = state
        if self.state == 0:
            self.__set_color('red')
        elif self.state == 1:
            self.__set_color(GREEN)
        else:
            self.__set_color('orange')
        self.update()

    def __set_color(self, color):
        self.setStyleSheet(
            "QRadioButton::indicator {width: 14px; height: 14px; border-radius: 7px;} QRadioButton::indicator:unchecked { background-color:" + color + "}")