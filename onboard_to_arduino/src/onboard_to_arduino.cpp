#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
//#include <string>
#include<stdio.h>

std_msgs::Int16MultiArray pwm;

void callback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
 pwm = *msg;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "onboard_to_arduino");

	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe<std_msgs::Int16MultiArray>
            ("/asl_gremlin1/arduino/cmd_pwm1", 10, callback);
    ros::Publisher pub = nh.advertise<std_msgs::Int16MultiArray>
            ("/asl_gremlin1/arduino/cmd_pwm", 10);

	//the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

//	std_msgs::Int16MultiArray  pwm;
	
	pwm.data.clear();
	
	pwm.data.push_back(0);
	pwm.data.push_back(0);

	 // wait for FCU connection
    while(ros::ok() ){
//	std::cout<<"pwm_callback"<<std::endl;
//	std::cout<<pwm_callback.data[0]<<std::endl;
/*	if (pwm_callback.data[0]!=NULL&&pwm_callback.data[1]!=NULL)
	{
	pwm.data[0] = 0;//pwm_callback.data[0];
	pwm.data[1] = 0;pwm_callback.data[1];
	}
	&pwm = pwm_callback
	else
	{*/
	// pwm.data[0] = 0;
	// pwm.data[1] = 0;

//	}

	pub.publish(pwm);
	ros::spinOnce();
	rate.sleep();

	}

	return 0;

	}

