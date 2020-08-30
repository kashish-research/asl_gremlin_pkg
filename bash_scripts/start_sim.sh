#!/bin/bash

source this_robot_name.sh

export ROS_MASTER_URI=http://192.168.10.2:11311


# give the robot a name, so all the nodes will be launched under it's namespace
robot_name=$ROBOT_NAME

if [ "$robot_name" == "" ]; then
	robot_name="asl_gremlin1"
fi

start_cmd='rostopic pub --once /'$robot_name'/start_sim std_msgs/Bool "data: true"'

eval $start_cmd
