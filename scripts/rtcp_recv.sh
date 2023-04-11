gst-launch-1.0 -v rtpbin name=rtpbin \
udpsrc address=127.0.0.1 port=50000 caps="application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264" ! rtpbin.recv_rtp_sink_0 \
udpsrc address=127.0.0.1 port=50001 caps="application/x-rtcp" ! rtpbin.recv_rtcp_sink_0 \
rtpbin. ! rtph264depay name=depay ! avdec_h264 ! videoconvert ! autovideosink