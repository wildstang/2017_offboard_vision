#!/bin/bash

cd /etc/init.d
mjpg_streamer -i "input_opencv.so -f 20 -r 864x480 -filter /usr/local/lib/mjpg-streamer/MyVision.so" -o "output_http.so -p 5801"
cd /home/pi
