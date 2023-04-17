import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst,GstRtp,GstRtsp,GObject
import yaml
import sys
import signal


def on_message(bus: Gst.Bus, message: Gst.Message, loop: GObject.MainLoop):
    mtype = message.type
    """
        Gstreamer Message Types and how to parse
  https://lazka.github.io/pgi-docs/Gst-1.0/flags.html#Gst.MessageType 
    """
    #print('received a message', mtype)
    if mtype == Gst.MessageType.EOS: 
        # Handle End of Stream 
        print("End of stream") 
        loop.quit()
    elif mtype == Gst.MessageType.ERROR: 
        # Handle Errors 
        err, debug = message.parse_error() 
        print(err, debug) 
        loop.quit()
    elif mtype == Gst.MessageType.WARNING: 
        # Handle warnings 
        err, debug = message.parse_warning() 
        print(err, debug) 
    elif mtype == Gst.MessageType.ELEMENT:
        print('element message')
    return True

class SignalHandler:
    def __init__(self, pipeline):
        self.pipeline = pipeline
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        print('You pressed Ctrl+C!')
        self.pipeline.set_state(Gst.State.NULL)
        sys.exit(0)

def main():
    conf = yaml.full_load(open(sys.argv[1]))

    try:
        print("GST_PLUGIN_PATH:", os.environ['GST_PLUGIN_PATH'])
    except:
        print("GST_PLUGIN_PATH not set")
    rtp_port = conf["rtp_port"]
    rtcp_port = conf["rtcp_port"]
    remote_ip = conf["remote_ip"]
    clock = "! clockoverlay valignment=top " if conf["gst_clock"] else ""
    source = conf["source"]

    # TODO check the source

    pipeline_string = f"rtpbin name=rtpbin ntp-sync=true {source} \
        ! qtdemux  \
        ! decodebin \
        ! videoconvert \
        ! x264enc tune=zerolatency bitrate=900 speed-preset=superfast \
        ! h264parse \
        ! rtph264pay \
        ! rtpbin.send_rtp_sink_0 rtpbin.send_rtp_src_0 \
        ! udpsink host={remote_ip} port={rtp_port} sync=true async=false rtpbin.send_rtcp_src_0 \
        ! udpsink host={remote_ip} port={rtcp_port} sync=false async=false"
    
    print("Gstreamer pipeline:", pipeline_string.replace("!","\n!"))
    pipeline = None
    try:
        Gst.init(None)
        pipeline = Gst.parse_launch(pipeline_string)
        pipeline.set_state(Gst.State.PLAYING)
        rtpbin = pipeline.get_by_name('rtpbin')
        session = rtpbin.emit('get-internal-session', 0)
    except Exception as e:
        print("\nERROR launching GSTREAMER. Check the GST_PLUGIN_PATH.\n")
        print("Error received:")
        print(e)
        return
    rtcp_interval = conf["rtcp_interval"] * 1e9 # in nanoseconds
    session.set_property("rtcp-min-interval", rtcp_interval)

    # https://lazka.github.io/pgi-docs/Gst-1.0/classes/Bus.html
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_message, None)

    signal_handler = SignalHandler(pipeline)

    loop = GObject.MainLoop() 
    try:
        print("Starting streaming...")
        loop.run()
    except: 
        loop.quit()
    print("Streamer quit.")

if __name__ == '__main__':
    main()
