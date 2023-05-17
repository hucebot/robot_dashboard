import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst,GstRtp,GstRtsp
import numpy as np
import datetime
import time
import os

class GStreamerFeed:    
    def __init__(self, conf):
        self.verbose = conf['gst_verbose']
        Gst.init(None)
        self.conf = conf
        self.init()

    def print(self, *args):
        if self.verbose: 
            print(*args)

    def init(self):
        # our general pipeline is that:
        # - we receive the data by RTP on rtp_port (UDP)
        # - we receive the data about the stream by RTCP (UDP) on rtcp_port at low frequency (default is 5 s) to get the network delay
        # - we take the rtp data, depayload it, decode it from h264, and get the image
        try:
            self.print("GST_PLUGIN_PATH:", os.environ['GST_PLUGIN_PATH'])
        except:
            self.print("GST_PLUGIN_PATH not set")
        rtp_port = self.conf["rtp_port"]
        rtcp_port = self.conf["rtcp_port"]
        timeout = int(self.conf["rtp_timeout"] * 1e9) # convert from s to nanoseconds
        clock = "! clockoverlay valignment=bottom " if self.conf["gst_clock"] else ""
        caps = "application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264"
        if self.conf['use_rtcp']:
            self.pipeline_string = f"rtpbin name=rtpbin buffer-mode=none udpsrc address=127.0.0.1 port={rtp_port} timeout={timeout} caps=\"{caps}\" name=src \
            ! rtpbin.recv_rtp_sink_0 udpsrc port={rtcp_port} caps=\"application/x-rtcp\"\
            ! rtpbin.recv_rtcp_sink_0 rtpbin. \
            ! rtph264depay name=depay \
            ! avdec_h264 \
            ! videoconvert \
            ! video/x-raw, format=RGB \
            {clock} \
            ! appsink name=sink emit-signals=true"
        else:
            self.pipeline_string = f"udpsrc port={rtp_port} name=src timeout={timeout} \
            ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 \
            ! rtph264depay ! h264parse ! decodebin ! videoconvert ! video/x-raw,format=RGB \
            ! queue ! appsink sync=true max-buffers=1 drop=true name=sink emit-signals=true"

            
        self.print("Gstreamer pipeline:", self.pipeline_string.replace("!","\n!"))

        try:
            self.pipeline = Gst.parse_launch(self.pipeline_string)
        except Exception as e:
            self.print("\nERROR launching GSTREAMER. Check GST_PLUGIN_PATH.\n")
            self.print("Error received:")
            self.print(e)
            return
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect("new-sample", self.__new_frame, self.appsink)

        if self.conf['use_rtcp']:
            self.rtpbin = self.pipeline.get_by_name('rtpbin')
            assert(self.rtpbin)
        self.print("Gstreamer pipeline elements (not in order):")
        for element in self.pipeline.iterate_elements():
            name = element.get_name()
            self.print("\t [{}]".format(name))
        if self.conf['use_rtcp']:
            self.print("Gstreamer rtpbin elements:")
            for element in self.rtpbin.iterate_elements():
                name = element.get_name()
                self.print("\t [{}]".format(name))
            self.session = self.rtpbin.emit('get-internal-session', 0)
            self.session.connect("on-ssrc-active", self.__on_ssrc)
        
        self.bus = self.pipeline.get_bus()
        self.frame_buffer = None
        self.jitter = 0.0
        self.bitrate = 0.0
        self.delay = 0.0
        self.playing = False

    def __on_ssrc(self, src, udata):
        self.__compute_network_delay()
        self.playing = True


    def __compute_network_delay(self):
        rtp_stats = self.session.get_property("stats")
        assert(rtp_stats)
        source_stats = rtp_stats["source-stats"]
        assert(source_stats)
        for i in range(0, len(source_stats)):
            is_sender = source_stats[i].get_boolean("is-sender").value
            if is_sender:
                #self.print(source_stats[i])
                self.jitter = source_stats[i].get_uint64("jitter").value
                self.bitrate = source_stats[i].get_uint64("bitrate").value
                ntp_time = source_stats[i].get_uint64("sr-ntptime").value
                ntp_seconds = ntp_time >> 32
                ntp_fraction = ntp_time & 0xffffffff
                unix_time = ntp_seconds - 2_208_988_800 # seconds between 1970 and 1900
                microseconds = (ntp_fraction * 1000000) // 0x100000000
                remote_time = unix_time * 1e6 + microseconds

                local_time = time.time_ns() // 1000
                delay = local_time - remote_time# in us
                if (delay < 0):
                    self.print("Please fix NTP: negative delays! =>", delay / 1000.0)
                self.delay = delay / 1000.0
                #self.print(delay)
                #self.print(unix_time * 1e6 + microseconds, "vs", local_time)
                #self.print("->", delay / 1000.0, " ms")
                # date_time = datetime.datetime.fromtimestamp(unix_time).strftime('%Y-%m-%d %H:%M:%S')
                # self.print("local:",  datetime.datetime.today().strftime('%Y-%m-%d %H:%M:%S.%f'))
                # self.print("remote:", date_time, microseconds)

    def getFrame(self):
        ret_frame = self.frame_buffer
        self.frame_buffer = None
        return ret_frame

    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)
      
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

    # return true of no problem
    def check_messages(self):
        message = self.bus.timed_pop_filtered(10000000, Gst.MessageType.ERROR |Gst.MessageType.STATE_CHANGED| Gst.MessageType.EOS | Gst.MessageType.ELEMENT |  Gst.MessageType.BUFFERING)
        #self.print("state:", self.pipeline.get_state())
        if message == None:
            return True
        #self.print("message", message.type)
        if message.type == Gst.MessageType.BUFFERING:
            self.print('buffering')
        if message.type == Gst.MessageType.EOS:
            self.pipeline.set_state(Gst.State.NULL)
            self.print("END OF STREAM")
        elif message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.print(f"Error: {err} ", debug)
            self.pipeline.set_state(Gst.State.NULL)
        elif message.type == Gst.MessageType.STATE_CHANGED:
            old_state, new_state, pending_state = message.parse_state_changed()
       #     self.print("State changed from {} to {}".format(old_state, new_state))
        elif message.type  == Gst.MessageType.ELEMENT:
            src_element = message.src
            if isinstance(src_element, Gst.Element) and src_element.get_name() == "src":
                if message.get_structure().get_name() == "GstUDPSrcTimeout":
                    self.print('timeout')
                    self.pipeline.set_state(Gst.State.NULL)
                    self.pipeline = None
                    self.playing = False
                    self.init()
                    self.start()
                    return False
        return True
    
    def isFrameReady(self):
        return not (self.frame_buffer is None)

    def interrupted(self):
        return False
    

