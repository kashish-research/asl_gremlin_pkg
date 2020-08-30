
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
   
    roverSpeedFiltered = 0;
    
    roverSpeedFilteredX = 0;
    
    roverSpeedFilteredY = 0;
  
}


void CollisionCone::computeCollisionConeY()
{
    
    double deltaT = vehicleState.pose.header.stamp.toSec() - vehicleStatePrevious.pose.header.stamp.toSec();
    
    double roverSpeedX =  (vehicleState.pose.point.x - vehicleStatePrevious.pose.point.x) / deltaT;
    
    double roverSpeedY =  (vehicleState.pose.point.y - vehicleStatePrevious.pose.point.y) / deltaT;
    
    if( ::isnan(roverSpeedX) || roverSpeedX>10)
    roverSpeedX = 0;
    
    if( ::isnan(roverSpeedY) || roverSpeedY>10)
    roverSpeedY = 0;
    
    roverSpeed = std::sqrt(roverSpeedX*roverSpeedX + roverSpeedY*roverSpeedY);
    
    roverSpeedVectorX[filterCurrentIndex] =  roverSpeedX;
	
	roverSpeedVectorY[filterCurrentIndex] =  roverSpeedY;
	
	roverSpeedFilteredX = accumulate(roverSpeedVectorX.begin(),
							roverSpeedVectorX.end(),0.0 )  / slidingFilterLength;
	
	roverSpeedFilteredY = accumulate(roverSpeedVectorY.begin(),
								roverSpeedVectorY.end(),0.0 ) / slidingFilterLength;
		
	roverSpeedFiltered = std::sqrt(roverSpeedFilteredX*roverSpeedFilteredX +
	                            roverSpeedFilteredY*roverSpeedFilteredY);
	
	/*	Debug Printing							 
	std::cout << "X: " ;
	
	for(int i=0; i < roverSpeedVectorX.size(); i++)
    {
   std::cout  << roverSpeedVectorX.at(i) << ' ';
   }
                   
   std::cout << " Sum: " << roverSpeedFilteredX << std::endl;
   
   std::cout << "Y: " ;
   
   for(int i=0; i < roverSpeedVectorY.size(); i++)
    {
   std::cout << roverSpeedVectorY.at(i) << ' ';
   }
   
    std::cout << " Sum: " << roverSpeedFilteredY << std::endl;
   
   std::cout << "\n";
	
	std::cout << std::to_string(roverSpeedFiltered) << " \n" ;
	*/

        filterCurrentIndex++;
	filterCurrentIndex %= slidingFilterLength;
    
    double distanceRoverWp = std::sqrt( std::pow(vehicleState.pose.point.x-obstacleState.pose.point.x,2) + 
                                        std::pow(vehicleState.pose.point.y-obstacleState.pose.point.y,2) );
       
    double LosAngle = std::atan2(obstacleState.pose.point.y-vehicleState.pose.point.y, 
                                        obstacleState.pose.point.x-vehicleState.pose.point.x);
   
    headingAngle =  std::atan2(roverSpeedFilteredY,roverSpeedFilteredX);      

    Vr =  - roverSpeedFiltered * std::cos(headingAngle-LosAngle);       
    Vtheta =   - roverSpeedFiltered * std::sin(headingAngle-LosAngle);   
 
	if(roverSpeedFiltered>0.05)
	{
    collisionConeY =  pow(distanceRoverWp,2)*pow(Vtheta,2)/( pow(Vtheta,2) + pow(Vr,2)) - pow(radiusSum,2);
    
    timeToCollision = -Vr*distanceRoverWp/( pow(Vr,2) + pow(Vtheta,2)); 
	}
	else
	{
		collisionConeY =  0;
    
    		timeToCollision = 0; 

	}   
 
	double refAccX, refAccY, refVelX, refVelY, refX, refY;
         
  if (collisionConeY<0 & Vr<0 & timeToCollision<timeToCollisionThrshold)

    { 
    
    double  refCollisionConeY = 0.01; double K = SIGN(timeToCollision)*0.2;
    
    double num = -K*(refCollisionConeY-collisionConeY)/(2*pow(distanceRoverWp,2)*Vtheta*Vr);
    
    double den = Vr*std::cos(headingAngle-LosAngle) + Vtheta*std::sin(headingAngle-LosAngle);
    
    aLat = num/den;

    refAccX = aLat*std::cos(headingAngle + M_PI/2);
    refAccY = aLat*std::sin(headingAngle + M_PI/2);
    
    refVelX = roverSpeedFilteredX + refAccX * deltaT;
    refVelY = roverSpeedFilteredY + refAccY * deltaT;


    }

    else

    {

    aLat=0; refAccX = 0; refAccY = 0;
    
    refVelX = 0.5*std::cos(vehicleState.heading*M_PI/180);
    refVelY = 0.5*std::sin(vehicleState.heading*M_PI/180);

    }

    refX = vehicleState.pose.point.x + refVelX * deltaT;
    refY = vehicleState.pose.point.y + refVelY * deltaT; 
        
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

double CollisionCone::getaLat() {return aLat;} 

double CollisionCone::getVr() {return Vr;} 

double CollisionCone::getHeadingAngle() {return headingAngle;} 


CollisionCone::~CollisionCone()
{

}


