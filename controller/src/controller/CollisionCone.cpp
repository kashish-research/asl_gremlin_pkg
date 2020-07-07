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

#include "controller/CollisionCone.h"

using namespace controller;
using namespace std;

void CollisionCone::obstacleStateCb(const asl_gremlin_msgs::VehicleState::ConstPtr& msg)
{
    obstacleState = *msg;
}

void CollisionCone::vehicleStateCb(const asl_gremlin_msgs::VehicleState::ConstPtr& msg)
{
    vehicleStatePrevious = vehicleState;
    vehicleState = *msg;
}

void CollisionCone::angVelCb(const asl_gremlin_msgs::MotorAngVel::ConstPtr& msg)
{
    bckstpCmdAngVel = *msg;

}

CollisionCone::CollisionCone(ros::NodeHandle& nh)
{
	obstacle_pose_sub = nh.subscribe<asl_gremlin_msgs::VehicleState>
            ("/asl_gremlin1/obstacle/pose", 10, &CollisionCone::obstacleStateCb,this);
    
   vehicle_pose_sub = nh.subscribe<asl_gremlin_msgs::VehicleState>
            ("/asl_gremlin1/state_feedback/selected_feedback", 10, &CollisionCone::vehicleStateCb,this);   
            
   cmd_ang_sub =  nh.subscribe<asl_gremlin_msgs::MotorAngVel>("/asl_gremlin1/controller/bckstp_cmd_angular_vel", 10, &CollisionCone::angVelCb,this); 
   
 	filterCurrentIndex = 0;     
   
    
}


void CollisionCone::computeCollisionConeY()
{
    
    double deltaT = vehicleState.pose.header.stamp.toSec() - vehicleStatePrevious.pose.header.stamp.toSec();
    
    double roverSpeedX =  (vehicleState.pose.point.x - vehicleStatePrevious.pose.point.x) / deltaT;
    
    double roverSpeedY =  (vehicleState.pose.point.y - vehicleStatePrevious.pose.point.y) / deltaT;
    
    roverSpeed = std::sqrt(roverSpeedX*roverSpeedX + roverSpeedY*roverSpeedY);
    
	roverSpeedVectorX.insert( roverSpeedVectorX.begin()+filterCurrentIndex, roverSpeedX);
	
	roverSpeedVectorY.insert( roverSpeedVectorY.begin()+filterCurrentIndex, roverSpeedY);
	
	roverSpeedFilteredX += accumulate(roverSpeedVectorX.begin(),
							roverSpeedVectorX.end(),0) / slidingFilterLength;
	
	roverSpeedFilteredY += accumulate(roverSpeedVectorY.begin(),
								roverSpeedVectorY.end(),0) / slidingFilterLength;
	
	roverSpeedFiltered = std::sqrt(roverSpeedFilteredX*roverSpeedFilteredX +
									 roverSpeedFilteredY*roverSpeedFilteredY);
	
	filterCurrentIndex = (filterCurrentIndex + 1) % 100;

    double distanceRoverWp = std::sqrt( std::pow(vehicleState.pose.point.x-obstacleState.pose.point.x,2) + 
                                        std::pow(vehicleState.pose.point.y-obstacleState.pose.point.y,2) );
       
    double LosAngle = std::atan2(obstacleState.pose.point.y-vehicleState.pose.point.y, 
                                        obstacleState.pose.point.x-vehicleState.pose.point.x);
   
    double headingAngle =  std::atan2(roverSpeedFilteredY,roverSpeedFilteredX);      

    double Vr =  - roverSpeedFiltered * std::cos(headingAngle-LosAngle);       
    double Vtheta =   - roverSpeedFiltered * std::cos(headingAngle-LosAngle);   
 
    collisionConeY =  pow(distanceRoverWp,2)*pow(Vtheta,2)/( pow(Vtheta,2) + pow(distanceRoverWp,2)) - pow(radiusSum,2);
    
    //std::cout <<  std::to_string(collisionConeY) << std::endl;
    
   // collisionConeY = -1;
    
    timeToCollision = -Vr*distanceRoverWp/( pow(Vr,2) + pow(Vtheta,2)); 
    
         
  if (collisionConeY<0 & timeToCollision>0 & timeToCollision<timeToCollisionThrshold)

    { 
    
    double  refCollisionConeY = 0.1; double K = SIGN(timeToCollision);
    
    double num = -K*(refCollisionConeY-collisionConeY)/(2*pow(distanceRoverWp,2)*Vtheta*Vr);
    
    double den = Vr*std::cos(headingAngle-LosAngle) + Vtheta*std::sin(headingAngle-LosAngle);
    
    aLat = num/den; 
        
    double refAccX = aLat*std::cos(headingAngle + M_PI/2);
    double refAccY = aLat*std::sin(headingAngle + M_PI/2);
    
    double refVelX = roverSpeedFilteredX + refAccX * deltaT;
    double refVelY = roverSpeedFilteredY + refAccY * deltaT;
    
    double refX = vehicleState.pose.point.x + refVelX * deltaT;
    double refY = vehicleState.pose.point.y + refVelY * deltaT;
    
    refCollAvoidTraj.header.stamp = ros::Time::now();
    refCollAvoidTraj.header.frame_id = "collision_avoidance";
    refCollAvoidTraj.x = refX; 
    refCollAvoidTraj.x_dot = refVelX;
    refCollAvoidTraj.x_ddot = refAccX;
    refCollAvoidTraj.y = refY;
    refCollAvoidTraj.y_dot = refVelY;
    refCollAvoidTraj.y_ddot = refAccY;
    refCollAvoidTraj.theta =  std::atan2( refVelY,refVelX );
    refCollAvoidTraj.theta_dot = 0;
    refCollAvoidTraj.theta_ddot = 0;
    
    }
 
 
  
  
    }
    
void CollisionCone::setTimeToCollsnThrshold(double tm_thrshhold) 
{timeToCollisionThrshold = tm_thrshhold;}

asl_gremlin_msgs::RefTraj CollisionCone::getRefTraj(){return refCollAvoidTraj;}

asl_gremlin_msgs::VehicleState CollisionCone::getVehicleState() { return vehicleState;}

double CollisionCone::getTimeToCollsnThrshold() {return timeToCollisionThrshold;}

asl_gremlin_msgs::MotorAngVel CollisionCone::getBckstpCmdVel() {return bckstpCmdAngVel;}

double CollisionCone::getCollisionConeY() { return collisionConeY; }

double CollisionCone::getTimeToCollision(){ return timeToCollision; }

double CollisionCone::getRoverSpeed() {return roverSpeed;} 

double CollisionCone::getRoverSpeedFiltered() {return roverSpeedFiltered;} 

CollisionCone::~CollisionCone()
{

}

