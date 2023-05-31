import sys
import time
import yaml

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *


from plot import Plot
from led import Led
from gstreamer_window import GstreamerWindow
import dark_style
import ping

# ROS2import rclpy
import rclpy
import rcl_interfaces
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rcl_interfaces.srv import GetParameters

class GstreamerRos2Thread(QThread):
    new_delay_data_signal =  pyqtSignal(float)
    new_bitrate_data_signal = pyqtSignal(float)
    new_packet = pyqtSignal(int)
    rtp_host = pyqtSignal(str)

    
    def __init__(self, conf):
        super().__init__()
        rclpy.init()
        self.conf = conf

        self.node = Node('gstreamer_client')
        self.sub_clock = self.node.create_subscription(Time, self.conf['gstreamer_topic'] + '/local_time', self.clock_callback, 10)
        self.network_delay = 0
        self.timer_new_packet = self.node.create_timer(0.5, lambda : self.new_packet.emit(2))
        self.get_params = self.node.create_client(GetParameters, self.conf['gstreamer_topic'] + '/get_parameters')
        self.sub_param_change = self.node.create_subscription(rcl_interfaces.msg.ParameterEvent, "/parameter_events", self.param_event_callback, 1)

        self.params_need_update = True

    def update_parameters(self):
        while not self.get_params.wait_for_service(timeout_sec=1.0):
            print('parameter service not available, waiting again...')
        req = GetParameters.Request()
        req.names = ["rtp_host", "rtp_port"]
        self.params = self.get_params.call_async(req)
        self.params_need_update = True


    def param_event_callback(self,msg: rcl_interfaces.msg.ParameterEvent):
        if msg.node == self.conf['gstreamer_topic']:
            self.update_parameters()


    def clock_callback(self, msg):
        t = time.time_ns()
        msg_ns = msg.sec *  1_000_000_000 + msg.nanosec
        self.network_delay = (t - msg_ns) /  1_000_000.0
        if (self.network_delay  < 0):
            self.logger.warning("Please fix NTP: negative delays! =>", self.network_delay)
        self.new_delay_data_signal.emit(self.network_delay)
        self.new_packet.emit(1)

    def run(self):
        self.update_parameters()
        while True:
            rclpy.spin_once(self.node)

            if self.params_need_update and self.params.done():
                self.params_need_update = False
                self.rtp_host = self.params.result().values[0].string_value
                self.rtp_port = self.params.result().values[1].integer_value
                print("RTP:", self.rtp_host, self.rtp_port)

class GstreamerPlots(QWidget):

    def __init__(self, conf, standalone=True):
        super(QWidget, self).__init__()
        self.conf = conf

        self.layout = QVBoxLayout()
        if standalone:
            self.button_quit = QPushButton('Quit')
            self.layout.addWidget(self.button_quit)
            self.button_quit.clicked.connect(QApplication.quit)

        
        self.led_ping = Led('Remote host: ' + conf['gstreamer_ip'])
        self.layout.addWidget(self.led_ping)
        self.led_ros2 = Led('Clock topic: /gstreamer_service/local_time')
        self.layout.addWidget(self.led_ros2)
        self.label_rtp = QLabel(self.conf['rtp_host'] + ':' + str(self.conf['rtp_port']))
        self.layout.addWidget(self.label_rtp)

        plot_names = ['ping', 'fps', 'delay']
        plot_labels = ['Ping [ms]', 'FPS', 'Delay [ms]']
        self.plots = {}
        for p,l in zip(plot_names,plot_labels):
            self.layout.addWidget(QLabel('<center><b>' + l + '</b></center>'))
            self.plots[p] = Plot()
            self.layout.addWidget(self.plots[p])

        self.setLayout(self.layout)

        # the gstreamer ping thread
        self.thread_ping_gstreamer = ping.PingThread(self.conf, self.conf['gstreamer_ip'])
        self.thread_ping_gstreamer.start()
        self.thread_ping_gstreamer.new_data.connect(self.plots['ping'].new_data)
        self.thread_ping_gstreamer.ok.connect(self.led_ping.set_state)

        # ros2 thread
        self.thread_ros2 = GstreamerRos2Thread(self.conf)
        self.thread_ros2.start()
        self.thread_ros2.new_delay_data_signal.connect(self.plots['delay'].new_data)
        self.thread_ros2.new_packet.connect(self.led_ros2.set_state)

        # conf
        self.plots['ping'].setYRange(0, self.conf['plot_ping_max'])
        self.plots['delay'].setYRange(0, self.conf['plot_delay_max'])
        self.plots['fps'].setYRange(0, self.conf['plot_fps_max'])

def main():
    
    if not "yaml" in sys.argv[-1]:
        print('usage: {} [--no-stdout-redirect] robot.yaml'.format(sys.argv[0]))
        sys.exit(1)
    conf = yaml.full_load(open(sys.argv[-1]))
    print("loaded: ", sys.argv[-1])
 
    app = QApplication(sys.argv)

    screen_size = QDesktopWidget().screenGeometry()

    plots = GstreamerPlots(conf)
    plots.setGeometry(0, 0,  int(screen_size.width()/6),
                      screen_size.height())
    plots.show()

    video = GstreamerWindow(conf, plots)
    video.setGeometry(int(screen_size.width()/2), 0,
                      int(screen_size.width()/2),
                      screen_size.height())
    #video.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
    video.show()

    dark_style.dark_style(app)
    app.exec_()



if __name__ == '__main__':
    main()

