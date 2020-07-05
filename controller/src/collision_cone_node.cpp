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
#include <trajectory_generation/TrajectorySwitcher.h>

using namespace  trajectory_generation;

using namespace controller;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_cone");
    
    ros::NodeHandle collision_nh;
    
    double rate = 10.0;
    
    ros::Rate loop_rate(rate);
    
    CollisionCone collision_cone_object(collision_nh);
    
    ros::Publisher cmd_ang_pub =  collision_nh.advertise<asl_gremlin_msgs::MotorAngVel>("/asl_gremlin1/controller/final_cmd_angular_vel", 10);   
    
    //// For debugging purposes--
    ros::Publisher collision_cone_y =  collision_nh.advertise<std_msgs::Float64>("/asl_gremlin1/collision_cone_y", 10);  
    ros::Publisher roverSpeedPublisher  =  collision_nh.advertise<std_msgs::Float64>("/asl_gremlin1/rover_speed", 10);  
    ////
    std::unique_ptr<ControllerBase<asl_gremlin_msgs::RefTraj, asl_gremlin_msgs::VehicleState>> controller = 
                            std::make_unique<BackSteppingController<asl_gremlin_msgs::RefTraj, asl_gremlin_msgs::VehicleState>>(collision_nh);

    double y,tm,tm_thrshhold, roverSpeed;
    
    std_msgs::Float64 collision_cone_y_value;
    std_msgs::Float64 roverSpeed_value;
    
	while(ros::ok())
    {
    
    collision_cone_object.computeCollisionConeY();
    y = collision_cone_object.getCollisionConeY();
    tm = collision_cone_object.getTimeToCollision();
    tm_thrshhold = collision_cone_object.getTimeToCollsnThrshold();
    roverSpeed = collision_cone_object.getRoverSpeed();
    
   
        if(y<0)// & tm>0 & tm<tm_thrshhold)
        {
        TrajectorySwitcher::collision_detected = true;
        //std::cout <<  collision_cone_object.getRefTraj().x << std::endl;
        
            controller->calculate_control_action( collision_cone_object.getRefTraj(),collision_cone_object.getVehicleState());
            
            cmd_ang_pub.publish(*(controller->get_control_action()));
        }
        else
        {
        TrajectorySwitcher::collision_avoided = true;
        cmd_ang_pub.publish( collision_cone_object.getBckstpCmdVel() );
        }
     
        cmd_ang_pub.publish( collision_cone_object.getBckstpCmdVel() );
        
        collision_cone_y_value.data = y;
        roverSpeed_value.data = roverSpeed;
        
        collision_cone_y.publish(collision_cone_y_value);
        roverSpeedPublisher.publish(roverSpeed_value);
        
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return EXIT_SUCCESS;

}




