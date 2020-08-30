/**
 * @brief Collision Cone
 * @file collision_cone_node.cpp
 * @author Kashish Dhal <kashish.dhal@mavs.uta.edu>
 */
/*
 * Copyright (c) 2020, kashishdhal
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/kashish-research/asl_gremlin_pkg/blob/master/LICENSE
 */
#include <ros/ros.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include "controller/CollisionCone.h"
#include <asl_gremlin_msgs/RefTraj.h>
#include <controller/BackSteppingController.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

using namespace controller;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_cone");
    
    ros::NodeHandle collision_nh;
    
    double rate = 50.0;
    
    ros::Rate loop_rate(rate);
    
    CollisionCone collision_cone_object(collision_nh);
    
    ros::Publisher cmd_ang_pub =  collision_nh.advertise<asl_gremlin_msgs::MotorAngVel>("/asl_gremlin1/controller/final_cmd_angular_vel", 10);   

    ros::Publisher traj_pub = collision_nh.advertise<asl_gremlin_msgs::RefTraj>("/asl_gremlin1/trajectory_generation/reference_trajectory", 10);
    
    //// For debugging purposes--
    ros::Publisher collision_cone_y =  collision_nh.advertise<std_msgs::Float64>("/asl_gremlin1/collision_cone_y", 10);  
    ros::Publisher roverSpeedPublisher  =  collision_nh.advertise<std_msgs::Float64>("/asl_gremlin1/rover_speed", 10);  
    ros::Publisher roverSpeedFilteredPublisher  =  collision_nh.advertise<std_msgs::Float64>("/asl_gremlin1/rover_speed_filtered", 10);    
    ros::Publisher time_to_collision_pub = collision_nh.advertise<std_msgs::Float64>("/asl_gremlin1/time_to_collision",10);  
    ros::Publisher a_lat_pub = collision_nh.advertise<std_msgs::Float64>("/asl_gremlin1/a_lat",10);
    ros::Publisher headingAngle_pub = collision_nh.advertise<std_msgs::Float64>("/asl_gremlin1/headingAngle",10);
    
////////////////////////////

	ros::Publisher collision_detected_pub = collision_nh.advertise<std_msgs::Bool>("/asl_gremlin1/collision_detected",10);	


    std::unique_ptr<ControllerBase<asl_gremlin_msgs::RefTraj, asl_gremlin_msgs::VehicleState>> controller = 
                            std::make_unique<BackSteppingController<asl_gremlin_msgs::RefTraj, asl_gremlin_msgs::VehicleState>>(collision_nh);


	double y,tm,a_lat,tm_thrshhold = 3,
	last_timestamp_collision_avoidance=0, Vr, headingAngle;
	
	bool collision_avoidance_started = false;

	collision_cone_object.setTimeToCollsnThrshold(tm_thrshhold);	
    
     //// For debugging purposes--
	double roverSpeed,roverSpeedFiltered;
    std_msgs::Float64 collision_cone_y_value, roverSpeed_value, 
    roverSpeedFiltered_value, time_to_collision,a_lat_value, headingAngle_value;
	///////////////////////////////////

	std_msgs::Bool collision_detected_data;
	collision_detected_data.data = false;

	while(ros::ok())
    {
    
    collision_cone_object.computeCollisionConeY();
    y = collision_cone_object.getCollisionConeY();
    tm = collision_cone_object.getTimeToCollision();
    roverSpeed = collision_cone_object.getRoverSpeed();
    roverSpeedFiltered = collision_cone_object.getRoverSpeedFiltered();
    a_lat = collision_cone_object.getaLat();
    headingAngle = collision_cone_object.getHeadingAngle();

    Vr = collision_cone_object.getVr();

    
   
        if(y<0 & Vr<0 & tm<tm_thrshhold)
        {
	collision_avoidance_started = true;
	last_timestamp_collision_avoidance = ros::Time::now().toSec(); 
        collision_detected_data.data = true;
	traj_pub.publish( collision_cone_object.getRefTraj() );
        		
        controller->calculate_control_action( collision_cone_object.getRefTraj(),collision_cone_object.getVehicleState());
            
        cmd_ang_pub.publish(*(controller->get_control_action()));
        }
        
        else if (collision_avoidance_started && Vr<0.1 )
	{

		collision_detected_data.data = true;
		
        	traj_pub.publish( collision_cone_object.getRefTraj() );
        
            controller->calculate_control_action( collision_cone_object.getRefTraj(),collision_cone_object.getVehicleState());


            cmd_ang_pub.publish(*(controller->get_control_action()));
 
		
	}
	
	else
        {
		collision_avoidance_started = false;
		collision_detected_data.data = false;       
        	cmd_ang_pub.publish( collision_cone_object.getBckstpCmdVel() );
        }
		
		
		//// For debugging purposes--
		time_to_collision.data = tm;
		time_to_collision_pub.publish(time_to_collision);        
        collision_cone_y_value.data = y;
        collision_cone_y.publish(collision_cone_y_value);
        roverSpeed_value.data = roverSpeed;
        roverSpeedPublisher.publish(roverSpeed_value);
        roverSpeedFiltered_value.data = roverSpeedFiltered;
        roverSpeedFilteredPublisher.publish(roverSpeedFiltered_value);
	a_lat_value.data = a_lat;
	a_lat_pub.publish(a_lat_value);
	collision_detected_pub.publish(collision_detected_data);
        headingAngle_value.data = headingAngle;
	headingAngle_pub.publish(headingAngle_value);
	//////////////////////////////////
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return EXIT_SUCCESS;

}




