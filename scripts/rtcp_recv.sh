# $0
gst-launch-1.0 -v \
rtpbin name=rtpbin \
udpsrc address=$1 port=5000 do-timestamp=true  caps="application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264" ! rtpbin.recv_rtp_sink_0 \
udpsrc address=$1 port=5001 caps="application/x-rtcp" ! rtpbin.recv_rtcp_sink_0 \
rtpbin. ! rtph264depay name=depay ! avdec_h264 ! videoconvert ! autovideosink \
rtpbin.send_rtcp_src_0 ! udpsink port=5002 host=$1 sync=false async=false