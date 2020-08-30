#!/bin/bash

source this_robot_name.sh

robot_name=$ROBOT_NAME

if [ "robot_name" == "" ]; then
    robot_name="asl_gremlin1"
fi

arm_cmd='rosservice call /'$robot_name'/mavros/cmd/arming/ "value: true"'

eval $arm_cmd
