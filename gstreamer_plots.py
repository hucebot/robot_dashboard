import sys
import time
import yaml
import subprocess

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *


from plot import Plot
from led import Led
#from gstreamer_window import GstreamerWindow
import dark_style
import ping
import netifaces

# ROS2import rclpy
import rclpy
import rcl_interfaces
from rclpy.node import Node
from builtin_interfaces.msg import Time
from std_msgs.msg import Float64

from rcl_interfaces.srv import *
from rcl_interfaces.msg import *
import rclpy.parameter


class GstreamerRos2Thread(QThread):
    new_delay_data_signal =  pyqtSignal(float)
    new_bitrate_data_signal = pyqtSignal(float)
    new_packet = pyqtSignal(int)
    rtp_host = pyqtSignal(str) 

    
    def __init__(self, conf):
        super().__init__()
        print('initializing ros2...')
        rclpy.init()
        print('ROS initialized')
        self.conf = conf

        print('creating node')
        self.node = Node('gstreamer_client')
        print('ok')

        self.sub_clock = self.node.create_subscription(Time, self.conf['gstreamer_topic'] + '/local_time', self.clock_callback, 10)
        self.network_delay = 0
        self.timer_new_packet = self.node.create_timer(0.5, lambda : self.new_packet.emit(2))
        self.get_params = self.node.create_client(GetParameters, self.conf['gstreamer_topic'] + '/get_parameters')
        self.set_params = self.node.create_client(SetParameters, self.conf['gstreamer_topic'] + '/set_parameters')
        self.sub_param_change = self.node.create_subscription(rcl_interfaces.msg.ParameterEvent, "/parameter_events", self.param_event_callback, 1)
        self.params_need_update = True

        self.pub_pan = self.node.create_publisher(Float64, self.conf['gstreamer_topic'] + '/pan', 1)
        self.pub_tilt = self.node.create_publisher(Float64, self.conf['gstreamer_topic'] + '/tilt', 1)

        print('before ntpsync')
        self.ntp_sync()
        print('npt sync ok')
        
        self.parameters_set = self.set_parameters() # set parameters to the ros node

    def ntp_sync(self):
        # we use popen so that this is asynchronous
        print('syncing clock (this might take a few secs...)')
        subprocess.Popen(['ntpdate', self.conf['ntp_server']])

    def set_pan(self, value):
        msg = Float64()
        msg.data = float(value)
        self.pub_pan.publish(msg)

    def set_tilt(self, value):
        msg = Float64()
        msg.data = float(value)
        self.pub_tilt.publish(msg)


    def set_parameters(self):
        print(netifaces.ifaddresses(self.conf['local_interface_name']))
        self.my_ip = netifaces.ifaddresses(self.conf['local_interface_name'])[2][0]['addr']
        if not self.set_params.wait_for_service(timeout_sec=1.0):
             return False
        req = SetParameters.Request()
        rtp_dest = rclpy.parameter.Parameter(name='rtp_dest', value=self.my_ip).to_parameter_msg()
        rtp_port = rclpy.parameter.Parameter(name='rtp_port', value=self.conf['rtp_port']).to_parameter_msg()
        ntp_server = rclpy.parameter.Parameter(name='ntp_server', value=self.conf['ntp_server']).to_parameter_msg()
        req.parameters = [rtp_dest, rtp_port, ntp_server]
        _ = self.set_params.call_async(req)
        return True


    def update_parameters(self):
        while not self.get_params.wait_for_service(timeout_sec=1.0):
            print('parameter service not available, waiting...')
        req = GetParameters.Request()
        req.names = ["rtp_port"]
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
        # get the current parameters
        #self.update_parameters()

        while True:
            rclpy.spin_once(self.node)
            time.sleep(0.01)
            if not self.parameters_set:
                self.parameters_set = self.set_parameters()

            # if self.params_need_update and self.params.done():
            #     self.params_need_update = False
            #     self.rtp_port = self.params.result().values[0].integer_value
            #     print("RTP:", self.rtp_host, self.rtp_port)

class GstreamerPlots(QWidget):

    def __init__(self, conf, standalone=True):
        super(QWidget, self).__init__()
        self.conf = conf

        self.layout = QVBoxLayout()
        if standalone:
            self.button_quit = QPushButton('Quit')
            self.layout.addWidget(self.button_quit)
            self.button_quit.clicked.connect(QApplication.quit)
            self.h_layout = QHBoxLayout()
            self.button_save_layout = QPushButton("Save window pos.")
            self.h_layout.addWidget(self.button_save_layout)
            self.button_save_layout_as = QPushButton("as...")
            self.h_layout.addWidget(self.button_save_layout_as)
            self.layout.addLayout(self.h_layout)

        self.led_ros2 = Led('Clock topic:'  + self.conf['gstreamer_topic'] + '/local_time')
        self.layout.addWidget(self.led_ros2)

        plot_names = ['fps', 'delay']
        plot_labels = ['FPS', 'Delay [ms]']
        self.plots = {}
        for p,l in zip(plot_names,plot_labels):
            self.layout.addWidget(QLabel('<center><b>' + l + '</b></center>'))
            self.plots[p] = Plot()
            self.layout.addWidget(self.plots[p])

        self.setLayout(self.layout)

        # ros2 thread
        print('creating ros2 thread...')
        self.thread_ros2 = GstreamerRos2Thread(self.conf)
        self.thread_ros2.start()
        self.thread_ros2.new_delay_data_signal.connect(self.plots['delay'].new_data)
        self.thread_ros2.new_packet.connect(self.led_ros2.set_state)
        print('ros ok')


        # conf
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

    # video = GstreamerWindow(conf, plots)
    # video.setGeometry(int(screen_size.width()/2), 0,
    #                   int(screen_size.width()/2),
    #                   screen_size.height())
    # #video.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
    # video.show()

    dark_style.dark_style(app)
    app.exec_()



if __name__ == '__main__':
    main()

