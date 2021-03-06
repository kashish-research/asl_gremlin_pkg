/**
 * @brief publish selected feedback node
 * @file feedback_selected.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
//#include <state_feedback/FeedbackSelected.h>
#include <ros/ros.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>

geometry_msgs::TransformStamped current_state;
std_msgs::Float64 yaw;

void state_cb(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	current_state = *msg;
}

void state_cb1(const std_msgs::Float64::ConstPtr& msg)
{
	yaw = *msg;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "feedback_selected");
	
    ros::NodeHandle feedback_nh;
    
    ros::Subscriber state_sub = feedback_nh.subscribe<geometry_msgs::TransformStamped>("/vicon/aslRover1/aslRover1",10,state_cb);
    ros::Subscriber state_sub1 = feedback_nh.subscribe<std_msgs::Float64>("/vicon/yaw",10,state_cb1);
    
    ros::Publisher local_pos_pub = feedback_nh.advertise<asl_gremlin_msgs::VehicleState>("/asl_gremlin1/state_feedback/selected_feedback",10);
    
    
    asl_gremlin_msgs::VehicleState pose;
    
    ros::Rate rate(50.0);
    
    int count = 0;
    
/**
    state_feedback::FeedbackSelected<3> feedback_method(feedback_nh);

    double rate = 10.0;

    if (!feedback_nh.getParam("sim/rate", rate))
    { ROS_WARN("Unable access parameter /%s/sim/rate, setting rate as 10Hz",
                ros::this_node::getNamespace().c_str()); }

    ros::Rate loop_rate(rate);

    
    ROS_INFO("\033[1;32mInitialized\033[0;m:= %s",ros::this_node::getName().c_str()); **/
    while(ros::ok())
    {
   //     feedback_method.get_gps_data();
       // feedback_method.get_enco_data();
 //       feedback_method.publish();
		 
		pose.pose.header.stamp = ros::Time::now();
    	pose.pose.header.seq = count;
    	pose.pose.header.frame_id = "vicon data";
		pose.pose.point.x = current_state.transform.translation.x;    
		pose.pose.point.y = current_state.transform.translation.y;
		pose.pose.point.z = current_state.transform.translation.z;
	    pose.heading = yaw.data;
     	local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
        ++count;
    }
}
