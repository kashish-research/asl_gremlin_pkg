#!/bin/bash

export ROS_MASTER_URI=http://192.168.10.2:11311
export ROSLAUNCH_SSH_UNKNOWN=1
source /home/gnclab/catkin_ws/devel/setup.sh
exec "$@"
