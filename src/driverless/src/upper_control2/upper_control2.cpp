#include"control_by_gps.h"



int main(int argc,char**argv)
{
	ros::init(argc,argv,"upper_control_node");
	
	Control_by_gps gps;
	
	//Control_by_lidar lidar;
 	
 	gps.run();
 	
 	//lidar.run();
 	
 	ros::spin();
 	
 	return 0;
}


