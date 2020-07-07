/**
 * @brief CollisionCone definitions
 * @file CollisionCone.cpp
 * @author Kashish Dhal <kashish.dhal@mavs.uta.edu>
 */
/*
 * Copyright (c) 2020, kashishdhal
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/kashish-research/asl_gremlin_pkg/blob/master/LICENSE
 */
 
#ifndef _controller_COLLISIONCONE_H_
#define _controller_COLLISIONCONE_H_

#include <vector>
#include <ros/ros.h>
#include <cmath>
#include <numeric>
#include <asl_gremlin_msgs/MotorPwm.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <std_msgs/Float32.h>

#define SIGN(x) (x>0?1:(x==0?0:-1))

using namespace std;

namespace controller{

class CollisionCone{

    private:
        asl_gremlin_msgs::VehicleState obstacleState;
        asl_gremlin_msgs::VehicleState vehicleState;
        asl_gremlin_msgs::VehicleState vehicleStatePrevious;
        asl_gremlin_msgs::MotorAngVel bckstpCmdAngVel;
        ros::Subscriber obstacle_pose_sub;
        ros::Subscriber vehicle_pose_sub;
        ros::Subscriber cmd_ang_sub;
        double roverSpeed;
	double roverSpeedFiltered;
	double roverSpeedFilteredX;
	double roverSpeedFilteredY;
        double collisionConeY;
        double timeToCollision;
        double timeToCollisionThrshold;
        double radiusSum=0.5; // radius of vehicle + radius of obstacle
        double aLat;
        asl_gremlin_msgs::RefTraj refCollAvoidTraj; 
        int slidingFilterLength=10;
	vector<double> roverSpeedVectorX{vector<double>(slidingFilterLength,0)};
	vector<double> roverSpeedVectorY{vector<double>(slidingFilterLength,0)};     
	int filterCurrentIndex;    
        

    public:
        CollisionCone(ros::NodeHandle& nh);
        void vehicleStateCb(const asl_gremlin_msgs::VehicleState::ConstPtr& msg);
        void obstacleStateCb(const asl_gremlin_msgs::VehicleState::ConstPtr& msg);  
        void angVelCb(const asl_gremlin_msgs::MotorAngVel::ConstPtr& msg);     
        void computeCollisionConeY();
        asl_gremlin_msgs::RefTraj getRefTraj();
        asl_gremlin_msgs::VehicleState getVehicleState();
        asl_gremlin_msgs::MotorAngVel getBckstpCmdVel();
        double getCollisionConeY();
        double getTimeToCollision();
        double getRoverSpeed();
        double getTimeToCollsnThrshold();
        double getRoverSpeedFiltered();
        void setTimeToCollsnThrshold(double tm_thrshhold);
        ~CollisionCone();
        
};

} //end namespace


#endif
