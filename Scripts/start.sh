#!/bin/bash

cd /etc/init.d
case "$1" in
p)
  ./mjpg_streamkick.sh start play
  cd /home/pi
;;
*)
  ./mjpg_streamkick.sh start
  cd /home/pi
;;
esac
