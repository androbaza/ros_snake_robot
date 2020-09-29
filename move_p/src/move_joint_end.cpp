#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <math.h>
//#include <iostream>
//using namespace std;
///robot/joint1_position_controller/state/process_value
float X;

void joint_angle_callback(const std_msgs::Float64 msg)
{
	//
	if(X<msg.data)
	{
		X = msg.data;
	}
	ROS_INFO("X is %f", X);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_end_joint");

	X = 0;

	ros::NodeHandle nh;

	ros::Publisher pub_end = nh.advertise<std_msgs::Float64>("/robot/joint5_position_controller/command", 100);

	ros::Subscriber sub_end = nh.subscribe("end_joint_angle", 1, joint_angle_callback);

	ros::Rate loop_rate(1);

	ros::Time startTime = ros::Time::now();

	//float temp = 0;

	while(ros::ok())
	{
		std_msgs::Float64 msg_to_send;

		msg_to_send.data = X;

		pub_end.publish(msg_to_send);

		ROS_INFO("position is %f", msg_to_send.data);

		ros::spinOnce();
 
		loop_rate.sleep();
	}
}
