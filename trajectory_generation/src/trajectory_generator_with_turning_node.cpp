/**
 * @brief trajectory_generator_with_turning_node
 * @file trajectory_generator_with_turning_node.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#include <trajectory_generation/MinimumJerkTrajectory.h>
#include <trajectory_generation/CircularTrajectory.h>
#include <trajectory_generation/WaypointSubscribe.h>
#include <trajectory_generation/TrajectorySwitcher.h>
#include <std_msgs/Bool.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <utility_pkg/utilities.h>

#include <ros/ros.h>
#include <ros/time.h>

#include <string>
#include <chrono>
#include <thread>
#include <memory>

using namespace trajectory_generation;

struct traj_params{
   double accel_max = 0.1;
    traj_params(ros::NodeHandle& nh){
        if (!nh.getParam("sim/max_accel",accel_max))
        { 
            ROS_WARN("Unable to access param '%s/sim/max_accel', setting to 0.1m/sec^2",
                    ros::this_node::getNamespace().c_str());
            accel_max = 0.1;
        }
    }
};

struct circle_params{
    double min_turn_rad = 2; // (m)
    double const_turn_vel = 0.5; // (m/sec)
    circle_params(ros::NodeHandle& nh){
        if (!nh.getParam("sim/min_turn_radius",min_turn_rad))
        {
            ROS_WARN("Unable to access param '%s/sim/min_turn_radius', setting to 2m",
                    ros::this_node::getNamespace().c_str());
            min_turn_rad = 2.0;
        }
        if (!nh.getParam("sim/const_turn_vel",const_turn_vel))
        {
            ROS_WARN("Unable to access param '/%s/sim/const_turn_vel', setting to 0.5m/sec",
                    ros::this_node::getNamespace().c_str());
            const_turn_vel = 0.5;
        }
    }
};

std_msgs::Bool collision_detected;
void colnDetSubCb(const std_msgs::Bool::ConstPtr& msg)
{
    collision_detected = *msg;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv , "trajectory_generation"); 

    ros::NodeHandle traj_nh;

    ros::Subscriber collision_detected_sub = traj_nh.subscribe<std_msgs::Bool>
            ("/asl_gremlin1/collision_detected", 10, colnDetSubCb);   

    traj_params params(traj_nh);
    circle_params params_circle(traj_nh);

    TrajectoryBase* traj_gen = nullptr;
    MinimumJerkTrajectory<traj_params>* min_jerk_traj = 
                                        new MinimumJerkTrajectory<traj_params>(traj_nh, &params);
    
    CircularTrajectory<circle_params>* circular_traj = 
                                        new CircularTrajectory<circle_params>(traj_nh, &params_circle);
    traj_gen = min_jerk_traj;

    WaypointSubscribe waypoint_stack(traj_nh);

    std::unique_ptr<TrajectorySwitcher> switch_trajectory = 
                            std::make_unique<TrajectorySwitcher>(traj_nh);

    asl_gremlin_pkg::SubscribeTopic<std_msgs::Bool> sim(traj_nh,"start_sim"); 
    
    std::string traj_pub_name;
    if(!traj_nh.getParam("trajectory/publisher_topic", traj_pub_name))
    { traj_pub_name = "trajectory_generation/reference_trajectory"; }

    ros::Publisher traj_pub = traj_nh.advertise<asl_gremlin_msgs::RefTraj>(traj_pub_name, 10);

    double rate = 10.0;
    if (!traj_nh.getParam("sim/rate", rate))
    {
        ROS_WARN("Unable to access parameter /%s/sim/rate, setting rate as 10Hz",
                    ros::this_node::getNamespace().c_str());
    }
    ros::Rate loop_rate(rate);

    bool updated_ini_params = false;
    bool aligned_rover = false;
    std::vector<double> waypoint(2,0);
    
    ROS_INFO("\033[1;32mInitialized\033[0;m:= %s",ros::this_node::getName().c_str());
    while(ros::ok())
    {
        if ( (sim.get_data())->data )
        {
            if (!waypoint_stack.received_waypoints())
            { ROS_ERROR("Mismatch waypoint size for (X,Y) or Waypoints are not specified"); }
            else
            {
                if ( !updated_ini_params )
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    waypoint_stack.reset_counter();
                    switch_trajectory->reset_vehicle_state();
                    ROS_INFO("\033[1;32mStarted\033[0;m:= Generating trajectory for given waypoints");

                    waypoint = waypoint_stack.get_current_waypoint();
                    switch_trajectory->change_next_desired_state(waypoint[0], waypoint[1]);

                    if (!switch_trajectory->current_hdg_within_tolerance_to_ref())
                    {
                        traj_gen = circular_traj;
                        switch_trajectory->change_switch_condition(trajSwitchCond::delta_theta_to_ref);
                        waypoint_stack.decrement_counter();
                        aligned_rover = true;
                    }
                    else
                    {
                        traj_gen = min_jerk_traj;
                        switch_trajectory->change_switch_condition(trajSwitchCond::dist_to_waypoint);
                        aligned_rover = false;
                    }

	           // traj_gen->set_ini_pose(0.0, 0.0);
		    traj_gen->set_current_pose_as_ini();
                    traj_gen->set_final_pose(waypoint[0], waypoint[1]);
                    traj_gen->calc_params();

                    updated_ini_params = true;
                }

                if ( switch_trajectory->need_to_switch_trajectory() )
                {
                    waypoint = waypoint_stack.get_next_waypoint();
                    
                    if (waypoint.size() == 1)
                    {
                        utility_pkg::stop_rover(ros::this_node::getNamespace());
                        ros::spinOnce(); 
                        switch_trajectory->reset_vehicle_state();
                        continue; 
                    }
                    switch_trajectory->change_next_desired_state(waypoint[0], waypoint[1]);
                    if (!switch_trajectory->current_hdg_within_tolerance_to_ref() && !aligned_rover)
                    {
                        traj_gen = circular_traj;
                        switch_trajectory->change_switch_condition(trajSwitchCond::delta_theta_to_ref);
                        waypoint_stack.decrement_counter();
                        aligned_rover = true;
                    }
                    else
                    {
                        traj_gen = min_jerk_traj;
                        switch_trajectory->change_switch_condition(trajSwitchCond::dist_to_waypoint);
                        aligned_rover = false;
                    }

                    traj_gen->set_current_pose_as_ini();
                    traj_gen->set_final_pose(waypoint[0], waypoint[1]);
                    traj_gen->calc_params();
                }

                traj_gen->generate_traj(ros::Time::now().toSec());
            }
        }
        else if (!(sim.get_data())->data && updated_ini_params)
        {
            ROS_INFO("\033[1;31mStopped\033[0;m:= Generating trajectory for given waypoints");
            updated_ini_params = false;
            switch_trajectory->reset_vehicle_state();
            waypoint_stack.reset_counter();
            aligned_rover = false;
        }

        if (traj_gen != nullptr && !collision_detected.data)
        { traj_pub.publish(*(traj_gen->get_trajectory())); }
        else if (!collision_detected.data)
        { traj_pub.publish(*(min_jerk_traj->get_trajectory())); }

        ros::spinOnce();
        loop_rate.sleep();
    }

    traj_gen = nullptr;
    delete min_jerk_traj;
    delete circular_traj;

    ROS_INFO("Gremlin signing off!");

    return EXIT_SUCCESS;
}
