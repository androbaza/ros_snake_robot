#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <math.h>
//#include <iostream>
//using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "square_wave");

	ros::NodeHandle nh;

	ros::Publisher pub_end = nh.advertise<std_msgs::Float64>("/robot/joint5_position_controller/command", 100);

	ros::Rate loop_rate(10);

	ros::Time startTime = ros::Time::now();

	//float temp = 0;

	while(ros::ok())
	{
		std_msgs::Float64 msg_to_send;

		msg_to_send.data = sin(ros::Time::now().toSec());
		
		pub_end.publish(msg_to_send);
		
		ROS_INFO("moving to position %f", msg_to_send.data);

		ros::spinOnce();
 
		loop_rate.sleep();
	}
}
