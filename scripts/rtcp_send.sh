gst-launch-1.0 -v \
rtpbin name=rtpbin \
filesrc location=$1 ! qtdemux  ! decodebin ! videoconvert  ! x264enc tune=zerolatency bitrate=900 speed-preset=superfast ! h264parse ! rtph264pay ! rtpbin.send_rtp_sink_0 \
rtpbin.send_rtp_src_0 ! udpsink host=127.0.0.1 port=50000 sync=true async=false \
rtpbin.send_rtcp_src_0 ! udpsink host=127.0.0.1 port=50001 sync=false async=false