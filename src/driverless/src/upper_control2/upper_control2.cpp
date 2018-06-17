#include"control_by_gps.h"
#include<geometry_msgs/Twist.h>


int main(int argc,char**argv)
{
	ros::init(argc,argv,"upper_control_node");
	
	//ros::Publisher control_pub;
	
	//control_pub= nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	
	Control_by_gps gps;
	
	//Control_by_lidar lidar;
 	
 	gps.run();
 	
 	//lidar.run();
 	
 	ros::spin();
 	
 	return 0;
}


