#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <std_msgs/Int16MultiArray.h>

 double left_speed_array[30],right_speed_array[30]; 
 int left_filter_index=0,right_filter_index=0;  

void left_speed_cb(const std_msgs::Float32::ConstPtr& msg)
{     left_speed_array[left_filter_index]=msg->data;
      left_filter_index = (left_filter_index + 1) % 30; 
}

void right_speed_cb(const std_msgs::Float32::ConstPtr& msg)
{  right_speed_array[right_filter_index]=msg->data; 
 right_filter_index = (right_filter_index + 1) % 30; }

std_msgs::Int16MultiArray pwm_arr;
double des_left_ang_speed = 0, des_right_ang_speed = 0;
void ang_vel_cb(const asl_gremlin_msgs::MotorAngVel::ConstPtr& msg)
{
 des_left_ang_speed = msg->wl;
 des_right_ang_speed = msg->wr; 
}



int main(int argc, char **argv)
{
 
    ros::init(argc, argv, "angular_speed_controller");
    ros::NodeHandle nh;

    std_msgs::Float32 left_angular_speed,right_angular_speed;
 
    for(int i=0;i<30;i++)
    {
	left_speed_array[i] = 0;
	right_speed_array[i] = 0;
    }
	
   ros::Subscriber left_raw_speed_sub = 
	nh.subscribe<std_msgs::Float32>
	("/asl_gremlin1/arduino/left_angular_speed",1,left_speed_cb);

    ros::Subscriber right_raw_speed_sub = 
	nh.subscribe<std_msgs::Float32>
	("/asl_gremlin1/arduino/right_angular_speed",1,right_speed_cb);

    ros::Subscriber ang_vel_sub = nh.subscribe<asl_gremlin_msgs::MotorAngVel>
            ("/asl_gremlin1/controller/final_cmd_angular_vel", 10, ang_vel_cb);

    ros::Publisher left_filtered_speed_pub = nh.advertise<std_msgs::Float32>
        ("/asl_gremlin1/filtered_left_angular_speed",1); 

    ros::Publisher right_filtered_speed_pub = nh.advertise<std_msgs::Float32> 
        ("/asl_gremlin1/filtered_right_angular_speed",1); 

    ros::Publisher pwm_pub = nh.advertise<std_msgs::Int16MultiArray>
				("/asl_gremlin1/arduino/cmd_pwm",20);

    pwm_arr.data.clear();
    pwm_arr.data.push_back(0);
    pwm_arr.data.push_back(0);                                     

    ros::Rate rate(250.0);

    int count=0; double K = 0.5;

    double error_term_right=0, error_term_left=0;

    while( ros::ok() )
    {
	left_angular_speed.data=0,right_angular_speed.data=0;
	
	for(int i=0;i<30;i++)
	{
	    left_angular_speed.data += left_speed_array[i];
	    right_angular_speed.data += right_speed_array[i];
	}

	left_angular_speed.data = left_angular_speed.data/30;
 	right_angular_speed.data = right_angular_speed.data/30;

	if(std::abs(left_angular_speed.data)<2)
	left_angular_speed.data = 0;

	if(std::abs(right_angular_speed.data)<2)
        right_angular_speed.data = 0;
	
	left_filtered_speed_pub.publish(left_angular_speed);
	right_filtered_speed_pub.publish(right_angular_speed);

	error_term_left += K*(des_left_ang_speed - left_angular_speed.data);
	error_term_right += K*(des_right_ang_speed - right_angular_speed.data);

	if(error_term_left>50){error_term_left=50;}
	else if(error_term_left<-50){error_term_left=-50;}

	if(error_term_right>50){error_term_right=50;}
        else if(error_term_right<-50){error_term_right=-50;}

	//error_term_left=0; error_term_right=0;

        if(count%10==0)
        {

	if(des_left_ang_speed>0)
	{
	if(des_left_ang_speed<2){
	pwm_arr.data[0] =0;}
	else if (des_left_ang_speed>2 & des_left_ang_speed<8){
	pwm_arr.data[0] = 60 + 0.3* error_term_left;}
	else {
        pwm_arr.data[0] = (des_left_ang_speed - 3.1794)/0.0655 + 
                                      error_term_left;}
	}
	else
	{
	if(des_left_ang_speed>-2){
        pwm_arr.data[0] =0;}
        else if (des_left_ang_speed<-2 & des_left_ang_speed>-8){
        pwm_arr.data[0] = -60 + 0.3* error_term_left;}
        else {
        pwm_arr.data[0] = (des_left_ang_speed + 3.1794)/0.0655 + 
                                      error_term_left;}
	}

	if(des_right_ang_speed>0)
	{
	if(des_right_ang_speed<2){
	pwm_arr.data[1] = 0;}
	else if (des_right_ang_speed>2 & des_right_ang_speed<8) {
        pwm_arr.data[1] = 75 + 0.3* error_term_right;  }
	else {
	pwm_arr.data[1] = (des_right_ang_speed - 3.1792)/0.0606 + 
			                        error_term_right;}
	}
	else
	{
	if(des_right_ang_speed>-2){
        pwm_arr.data[1] = 0;}
        else if (des_right_ang_speed<-2 & des_right_ang_speed>-8) {
        pwm_arr.data[1] = -75 +  0.3*error_term_right;  }
        else {
        pwm_arr.data[1] = (des_right_ang_speed + 3.1792)/0.0606 + 
                                                error_term_right;}
	}


        pwm_pub.publish(pwm_arr);
	}
	count++;

	ros::spinOnce();
        rate.sleep();
        
    }
              
            
    return 0;           
  
 }


