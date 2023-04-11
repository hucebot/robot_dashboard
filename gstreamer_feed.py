import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst,GstRtp
import numpy as np
import json
import datetime
import struct

# https://stackoverflow.com/questions/49858346/how-te-retrieve-stream-statistics-in-gstreamer
# https://gist.github.com/hum4n0id/cda96fb07a34300cdb2c0e314c14df0a#send-a-test-video-with-h264-rtp-stream
# https://stackoverflow.com/questions/39565204/how-to-make-rtpjitterbuffer-work-on-a-stream-without-timestamps

# gst-launch-1.0 filesrc location=/Users/jmouret/Movies/talos/talos_teleop_10min.MP4  ! qtdemux  ! decodebin ! videoconvert ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5000

# to test:
# videotestsrc pattern=ball ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5000

class GStreamerFeed:    
    def __init__(self, launch:str):
        Gst.init(None)
        self.launch = launch
        self.init()

    def init(self):
        self.pipeline = Gst.parse_launch(self.launch)
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect("new-sample", self.__new_frame, self.appsink)

        self.rtpbin = self.pipeline.get_by_name('rtpbin')
        assert(self.rtpbin)
        print("Gstreamer pipeline:")
        for element in self.pipeline.iterate_elements():
            name = element.get_name()
            print("\t [{}]".format(name))
        print("Gstreamer rtpbin:")
        for element in self.rtpbin.iterate_elements():
            name = element.get_name()
            print("\t [{}]".format(name))

#        self.jitter_buffer = self.pipeline.get_by_name('jitter_buffer')
 #       assert(self.jitter_buffer)

  #      self.depay = self.pipeline.get_by_name('depay')
   #     assert(self.depay)
    #    sink_pad = self.depay.get_static_pad("sink")
        # def pad_probe(pad, info):
        #     print('probe')
        #     buffer = info.get_buffer()
        #     (result, mapinfo) = buffer.map(Gst.MapFlags.READ)
        #     print(info.rtptimestamp())
            
        #     # Process the RTP packet here
        #     return Gst.PadProbeReturn.OK
        # sink_pad.add_probe(Gst.PadProbeType.BUFFER, pad_probe)


        self.session = self.rtpbin.emit('get-internal-session', 0)
        def test(buffer):
            print('receiving rtcp')
        self.session.connect("on-receiving-rtcp", test)
        self.bus = self.pipeline.get_bus()
        self.frame_buffer = None
        self.jitter = 0.0
        self.bitrate = 0.0

    def getFrame(self):
        ret_frame = self.frame_buffer
        self.frame_buffer = None
        return ret_frame

    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)
      
    def __to_numpy(self, sample):
        buf = sample.get_buffer()
        # For your own clock slaving, you could just set buffer-mode=none in rtpjitterbuffer. Then you will get the RTP timestamps as PTS and the arrival timestamps as DTS
        #print(buf.pts / Gst.SECOND, buf.dts / Gst.SECOND)
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
        dts = frame.get_buffer().dts / Gst.SECOND
        pts = frame.get_buffer().pts / Gst.SECOND
        print("dts:", dts, " pts:", pts)
        rtp_stats = self.session.get_property("stats")
        drops = rtp_stats.get_uint("rtx-drop-count")
        source_stats = rtp_stats["source-stats"]
        
#        print("source stats[0]:", source_stats[0])
        for i in range(0, len(source_stats)):
            is_sender = source_stats[i].get_boolean("is-sender").value
            if is_sender:
                print(source_stats[i])
                self.jitter = source_stats[i].get_uint64("rb-jitter").value
                self.bitrate = source_stats[i].get_uint64("bitrate").value
                ntp_time = source_stats[i].get_uint64("sr-ntptime").value
                rtp_time = source_stats[i].get_uint("sr-rtptime").value
                is_sender = source_stats[i].get_boolean("is-sender").value
                have_sr = source_stats[i].get_boolean("sent-rb").value
                print('have sr', have_sr)
                print("is sender:",is_sender)
                ntp_seconds = ntp_time >> 32
                ntp_fraction = ntp_time & 0xffffffff
                print(ntp_seconds)
                unix_time = ntp_seconds - 2208988800 # int((ntp_time - ntp_epoch.timestamp()) // 1)
                microseconds = (ntp_fraction * 1000000) // 0x100000000

                rtp_time_scaled = rtp_time * 90000.0 / Gst.SECOND
                # Convert the Unix timestamp to a human-readable date and time
                date_time = datetime.datetime.fromtimestamp(unix_time).strftime('%Y-%m-%d %H:%M:%S')
                print(date_time, microseconds,rtp_time_scaled + dts)

    #     if len(source_stats) > 1:
    #         self.bitrate = max(source_stats[0].get_uint64("bitrate").value, source_stats[1].get_uint64("bitrate").value)
    #         self.jitter = max(source_stats[0].get_uint64("jitter").value, source_stats[1].get_uint64("jitter").value)
    #        # print("j:", self.jitter)
    #     if len(source_stats) == 1:
    #         self.bitrate = source_stats[0].get_uint64("bitrate").value
    #    # jitter_stats = self.jitter_buffer.get_property("stats")
       # self.jitter = jitter_stats.get_uint64("avg-jitter").value
        #_, position = self.pipeline.query_position(Gst.Format.TIME)
        #print("position:",position * 1e-9)
      
        #print(self.jitter)
        #print(jitter_stats)
        # application/x-rtp-jitterbuffer-stats, num-pushed=(guint64)2125, num-lost=(guint64)0, num-late=(guint64)0, num-duplicates=(guint64)0, avg-jitter=(guint64)39, rtx-count=(guint64)0, rtx-success-count=(guint64)0, rtx-per-packet=(double)0, rtx-rtt=(guint64)0;

        # print(source_stats[0])
        #application/x-rtp-source-stats, ssrc=(uint)3654702140, internal=(boolean)false, validated=(boolean)true, received-bye=(boolean)false, is-csrc=(boolean)false, is-sender=(boolean)true, seqnum-base=(int)-1, clock-rate=(int)-1, rtp-from=(string)127.0.0.1:56297, octets-sent=(guint64)0, packets-sent=(guint64)0, octets-received=(guint64)814033, packets-received=(guint64)6487, bytes-received=(guint64)1073513, bitrate=(guint64)580969, packets-lost=(int)-1, jitter=(uint)0, sent-pli-count=(uint)0, recv-pli-count=(uint)0, sent-fir-count=(uint)0, recv-fir-count=(uint)0, sent-nack-count=(uint)0, recv-nack-count=(uint)0, recv-packet-rate=(uint)0, have-sr=(boolean)false, sr-ntptime=(guint64)0, sr-rtptime=(uint)0, sr-octet-count=(uint)0, sr-packet-count=(uint)0, sent-rb=(boolean)true, sent-rb-fractionlost=(uint)0, sent-rb-packetslost=(int)-1, sent-rb-exthighestseq=(uint)6021, sent-rb-jitter=(uint)0, sent-rb-lsr=(uint)0, sent-rb-dlsr=(uint)0, have-rb=(boolean)false, rb-ssrc=(uint)0, rb-fractionlost=(uint)0, rb-packetslost=(int)0,π rb-exthighestseq=(uint)0, rb-jitter=(uint)0, rb-lsr=(uint)0, rb-dlsr=(uint)0, rb-round-trip=(uint)0;
        #print(bitrate, bitrate2)
        
        self.frame_buffer = self.__to_numpy(frame)
        return Gst.FlowReturn.OK

    # return true of no problem
    def check_messages(self):
        message = self.bus.timed_pop_filtered(10000000, Gst.MessageType.ERROR |Gst.MessageType.STATE_CHANGED| Gst.MessageType.EOS | Gst.MessageType.ELEMENT)
        #print("state:", self.pipeline.get_state())
        if message == None:
            return True
        if message.type == Gst.MessageType.EOS:
            self.pipeline.set_state(Gst.State.NULL)
            print("END OF STREAM")
        elif message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err} ", debug)
            self.pipeline.set_state(Gst.State.NULL)
        elif message.type == Gst.MessageType.STATE_CHANGED:
            old_state, new_state, pending_state = message.parse_state_changed()
            #print("State changed from {} to {}".format(old_state, new_state))
        elif message.type  == Gst.MessageType.ELEMENT:
            src_element = message.src
            if isinstance(src_element, Gst.Element) and src_element.get_name() == "src":
                if message.get_structure().get_name() == "GstUDPSrcTimeout":
                    print('timeout')
                    self.pipeline.set_state(Gst.State.NULL)
                    self.pipeline = None
                    self.init()
                    self.start()
                    return False
        return True
    
    def isFrameReady(self):
        return not (self.frame_buffer is None)

    def interrupted(self):
        return False