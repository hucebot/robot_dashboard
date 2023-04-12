# $1 = file
# $2 = IP
gst-launch-1.0 -v \
rtpbin name=rtpbin ntp-sync=true \
filesrc location=$1 ! qtdemux  ! decodebin ! videoconvert  ! x264enc tune=zerolatency bitrate=900 speed-preset=superfast ! h264parse ! rtph264pay ! rtpbin.send_rtp_sink_0 \
rtpbin.send_rtp_src_0 ! udpsink host=$2 port=5000 sync=true async=false \
rtpbin.send_rtcp_src_0 ! udpsink host=$2 port=5001 sync=false async=false \
udpsrc address=$2 port=5002 caps="application/x-rtcp" ! rtpbin.recv_rtcp_sink_0 
