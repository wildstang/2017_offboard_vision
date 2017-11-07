#!/bin/bash

sleep 5
cd /home/pi/vision/test
make
make install
cd /etc/init.d
mjpg_streamer -i "input_opencv.so -f 15 -r 800x600 -filter /usr/local/lib/mjpg-streamer/MyVision.so" -o "output_http.so -p 5000"
