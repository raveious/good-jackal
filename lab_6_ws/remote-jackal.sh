#!/bin/sh

if [ "$#" -eq 1 ]; then
  export ROS_MASTER_URI=http://$1:11311   # Hostname for Jackal
  export ROS_IP="$(ifconfig | grep -A 1 'wlan0' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)"
else
  export ROS_MASTER_URI=http://localhost:11311
  unset ROS_IP
fi
