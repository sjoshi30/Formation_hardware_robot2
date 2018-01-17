#!/usr/bin/env sh

export ROS_HOSTNAME="192.168.0.103"
export ROS_ID="192.168.0.103"
. ~/catkin_ws/devel/setup.sh
exec "$@" 
