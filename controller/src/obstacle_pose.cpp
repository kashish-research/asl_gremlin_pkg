/**
 * @brief Commanded PWM to Arduino PWM
 * @file cmd_pwm_to_arduino_pwm.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#include <ros/ros.h>
#include <asl_gremlin_msgs/VehicleState.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_pose_publisher");

    ros::NodeHandle nh;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    ros::Publisher pub = nh.advertise<asl_gremlin_msgs::VehicleState>
            ("/asl_gremlin1/obstacle/pose", 10);    
                     
        int count = 1;  
    
    asl_gremlin_msgs::VehicleState pose;

	while(ros::ok())
    {   
        pose.pose.header.stamp = ros::Time::now();
        pose.pose.header.seq=count;
        pose.pose.header.frame_id = "Inertial Frame";  
        pose.pose.point.x = 1;
        pose.pose.point.y = 1;
        pose.pose.point.z = 0;
        pose.heading = 0;
    	pub.publish(pose);
        count++;
        ros::spinOnce();
        rate.sleep();
    }
    

}
