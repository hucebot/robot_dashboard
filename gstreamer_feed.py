import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst,GstRtp
import numpy as np
import json
# https://stackoverflow.com/questions/49858346/how-te-retrieve-stream-statistics-in-gstreamer
# https://gist.github.com/hum4n0id/cda96fb07a34300cdb2c0e314c14df0a#send-a-test-video-with-h264-rtp-stream
# https://stackoverflow.com/questions/39565204/how-to-make-rtpjitterbuffer-work-on-a-stream-without-timestamps

# gst-launch-1.0 filesrc location=/Users/jmouret/Movies/talos/talos_teleop_10min.MP4  ! qtdemux  ! decodebin ! videoconvert ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5000

# to test:
# videotestsrc pattern=ball ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5000

class GStreamerFeed:    
    def __init__(self, launch:str):
        # everything is delayed for connect
        Gst.init(None)
        #self.player = Gst.parse_launch('filesrc location=/Users/jmouret/Movies/teleop-short_1min.mp4 name=src ! decodebin !  videoconvert ! video/x-raw, format=RGB !appsink name=sink emit-signals=true')
        self.pipeline = Gst.parse_launch(launch)
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect("new-sample", self.__new_frame, self.appsink)

        self.rtpbin = self.pipeline.get_by_name('rtp')
        assert(self.rtpbin)
        
        self.session = self.rtpbin.emit('get-internal-session',0)
        
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.enable_sync_message_emission()
        self.bus.connect("message", self.__on_message)
        self.bus.connect("sync-message::element", self.__on_sync_message)
        self.bus.connect('message::qos', self.__on_qos)


      
        self.frame_buffer = None

    def on_new_rtp_packet(rtpbin, sessid, payload_type, packet):
        ssrc = packet.get_ssrc()
        internal_session = rtpbin.emit('get-internal-session', ssrc)
        stats = internal_session.get_property('stats')
        print(stats)

    def getFrame(self):
        ret_frame = self.frame_buffer
        self.frame_buffer = None
        return ret_frame

    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        str = Gst.debug_bin_to_dot_data(self.pipeline,  Gst.DebugGraphDetails.ALL)
        f = open('test.dot', 'w')
        f.write(str)

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
        stats = self.session.get_property("stats")
        drops = stats.get_uint("rtx-drop-count")
        source_stats = stats["source-stats"]
        bitrate = source_stats[0].get_uint64("bitrate")
        # print(source_stats[0])
        #application/x-rtp-source-stats, ssrc=(uint)3654702140, internal=(boolean)false, validated=(boolean)true, received-bye=(boolean)false, is-csrc=(boolean)false, is-sender=(boolean)true, seqnum-base=(int)-1, clock-rate=(int)-1, rtp-from=(string)127.0.0.1:56297, octets-sent=(guint64)0, packets-sent=(guint64)0, octets-received=(guint64)814033, packets-received=(guint64)6487, bytes-received=(guint64)1073513, bitrate=(guint64)580969, packets-lost=(int)-1, jitter=(uint)0, sent-pli-count=(uint)0, recv-pli-count=(uint)0, sent-fir-count=(uint)0, recv-fir-count=(uint)0, sent-nack-count=(uint)0, recv-nack-count=(uint)0, recv-packet-rate=(uint)0, have-sr=(boolean)false, sr-ntptime=(guint64)0, sr-rtptime=(uint)0, sr-octet-count=(uint)0, sr-packet-count=(uint)0, sent-rb=(boolean)true, sent-rb-fractionlost=(uint)0, sent-rb-packetslost=(int)-1, sent-rb-exthighestseq=(uint)6021, sent-rb-jitter=(uint)0, sent-rb-lsr=(uint)0, sent-rb-dlsr=(uint)0, have-rb=(boolean)false, rb-ssrc=(uint)0, rb-fractionlost=(uint)0, rb-packetslost=(int)0, rb-exthighestseq=(uint)0, rb-jitter=(uint)0, rb-lsr=(uint)0, rb-dlsr=(uint)0, rb-round-trip=(uint)0;
        print(bitrate)
        frame = sink.emit("pull-sample")
        self.frame_buffer = self.__to_numpy(frame)
        return Gst.FlowReturn.OK

    
    def __on_message(self, bus, message):
        if message.type == Gst.MessageType.EOS:
            self.pipeline.set_state(Gst.State.NULL)
        elif message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err} ", debug)
            self.pipeline.set_state(Gst.State.NULL)
        

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

