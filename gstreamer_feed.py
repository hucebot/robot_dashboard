from gi.repository import Gst
import numpy as np

import gi
gi.require_version('Gst', '1.0')

# https://stackoverflow.com/questions/49858346/how-te-retrieve-stream-statistics-in-gstreamer
# https://gist.github.com/hum4n0id/cda96fb07a34300cdb2c0e314c14df0a#send-a-test-video-with-h264-rtp-stream
# https://stackoverflow.com/questions/39565204/how-to-make-rtpjitterbuffer-work-on-a-stream-without-timestamps

# to test:
# videotestsrc pattern=ball ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5000

class GStreamerFeed:
    def __init__(self):
        Gst.init(None)
        #self.player = Gst.parse_launch('filesrc location=/Users/jmouret/Movies/teleop-short_1min.mp4 name=src ! decodebin !  videoconvert ! video/x-raw, format=RGB !appsink name=sink emit-signals=true')
        self.player = Gst.parse_launch('udpsrc port=5000 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw, format=RGB !appsink name=sink emit-signals=true')
        self.appsink = self.player.get_by_name('sink')
        self.appsink.connect("new-sample", self.__new_frame, self.appsink)

        bus = self.player.get_bus()
        bus.add_signal_watch()
        bus.enable_sync_message_emission()
        bus.connect("message", self.__on_message)
        bus.connect("sync-message::element", self.__on_sync_message)
        bus.connect('message::qos', self.__on_qos)

        self.frame_buffer = None
        bus.emit('message::qos',Gst.Message('xxxxx'))

    def getFrame(self):
        ret_frame = self.frame_buffer
        self.frame_buffer = None
        return ret_frame

    def start(self):
        self.player.set_state(Gst.State.PLAYING)

    def __on_qos(self, bus, msg):
        print('Qos Message:', msg.parse_qos())
        

    def __to_numpy(self, sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()
        arr = np.ndarray(
            (caps.get_structure(0).get_value('height'),
             caps.get_structure(0).get_value('width'),
             3),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8)
        return arr


    def __new_frame(self, sink, data):
        frame = sink.emit("pull-sample")
        self.frame_buffer = self.__to_numpy(frame)
        return Gst.FlowReturn.OK

    def __on_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS:
            self.player.set_state(Gst.State.NULL)
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err} ", debug)
            self.player.set_state(Gst.State.NULL)

    def __on_sync_message(self, bus, message):
        if message.get_structure().get_name() == 'prepare-window-handle':
            win_id = self.windowId
            imagesink = message.src
            imagesink.set_property("force-aspect-ratio", True)
            # if not window id then create new window
            if win_id is None:
                win_id = self.movie_window.get_property('window').get_xid()
            imagesink.set_window_handle(win_id)

    def isFrameReady(self):
        return not (self.frame_buffer is None)

