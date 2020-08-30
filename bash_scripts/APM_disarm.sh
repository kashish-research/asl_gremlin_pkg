#!/bin/bash

source this_robot_name.sh

robot_name=$ROBOT_NAME

if [ "robot_name" == "" ]; then
    robot_name="asl_gremlin1"
fi

disarm_cmd='rosservice call /'$robot_name'/mavros/cmd/arming/ "value: false"'

eval $disarm_cmd
