#!/bin/bash
unset ROS_MASTER_URI
unset ROS_IP

export ROS_MASTER_URI=http://blurr.local:11311

# for Ethernet
export ROS_IP=$(ip -4 addr show enp0s31f6 | grep -oP "(?<=inet ).*(?=/)")

if [ "$#" -ne 1 ]; then
# for WIFI
 export ROS_IP=$(ip -4 addr show wlp2s0 | grep -oP "(?<=inet ).*(?=/)")
fi

