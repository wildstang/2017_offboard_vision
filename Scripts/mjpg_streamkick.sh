### BEGIN INIT INFO
# Provides:          dovecot
# Required-Start:    $local_fs $network
# Required-Stop:     $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: dovecot
# Description:       dovecot pop & imap daemon
### END INIT INFO

#! /bin/sh
# /etc/init.d/mjpg_streamkick

# Carry out specific functions when asked to by the system
case "$1" in
  start)
    echo "Starting mjpg_streamkick script"
    export LD_LIBRARY_PATH=/usr/local/lib/mjpg-streamer
    uvcdynctrl -d video0 --set="Focus, Auto" 0
    uvcdynctrl -d video0 --set="Focus (absolute)" 0
    uvcdynctrl -d video0 --set="Exposure, Auto" 1
    uvcdynctrl -d video0 --set="Exposure (Absolute)" 17
    uvcdynctrl -d video0 --set="Saturation" 255
    uvcdynctrl -d video0 --set="gain" 128
    uvcdynctrl -d video0 --set="White Balance Temperature, Auto" 0
    uvcdynctrl -d video0 --set="White Balance Temperature" 2680
      case "$2" in
        play)
          echo "PLAYBACK mode = ON"
          mjpg_streamer -i "input_opencv.so -f 20 -r 864x480 -filter /usr/local/lib/mjpg-streamer/Playback.so" -o "output_http.so -p 5801"
          ;;
        *)
          echo "PLAYBACK mode = OFF"
          mjpg_streamer -i "input_opencv.so -f 20 -r 864x480 -filter /usr/local/lib/mjpg-streamer/MyVision.so" -o "output_http.so -p 5801"
          ;;
      esac
    #mjpg_streamer -i "input_opencv.so -sa 255 -ex 17 -f 20 -r 864x480 -filter /usr/local/lib/mjpg-streamer/MyVision.so" -o "output_http.so -p 5801"
    #
    #mjpg_streamer -i "input_opencv.so -f 20 -r 864x480 -filter /usr/local/lib/mjpg-streamer/MyVision.so" -o "output_http.so -p 5801"&
    #sleep 1
    #v4l2-ctl -d /dev/video0 -c exposure_auto=1
    #sleep 1
    #v4l2-ctl -d /dev/video0 -c exposure_absolute=17
    #sleep 1
    #v4l2-ctl -d /dev/video0 -c saturation=255
    #sleep 1
    #v412-ctl -d /dev/video0 -c focus_auto=0
    #sleep 1
    #v412-ctl -d /dev/video0 -c focus_absolute=0

    ;;
  stop)
    echo "Stopping webcam script"
    killall mjpg_streamer
    ;;
  *)
    echo "Usage: /etc/init.d/mjpg_streamkick {start|stop}"
    exit 1
    ;;
esac

exit 0

