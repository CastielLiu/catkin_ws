#include"ros/ros.h"
#include"ps_gps/gps.h"
#include"../include/ps_gps/ps_type.h"
#include<string.h>

void serial_callback(const ps_gps::gps& GPS)
{
		
	ROS_INFO("GPS MESSAGE RECEIVE");
	printf("latitude:  [%.10lf]\nlongitude: [%.10lf]\naltitude:  [%.10lf]\n", 
	 		GPS.lat, GPS.lon, GPS.hei);
	 		
	printf("velocity_n:  [%.10lf]\nvelocity_e: [%.10lf]\nvelocity_g:  [%.10lf]\n", 
	 		GPS.vel_n, GPS.vel_e, GPS.vel_g);
	
	printf("acceleration_y:  [%.10lf]\nangular_velocity_y: [%.10lf]\n\n", 
	 		GPS.acc_y, GPS.ang_y);
	
}


int main(int argc, char **argv)
{
    
	ros::init(argc, argv, "ps_receive_test");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("gps_serial_node", 1000, serial_callback);
	
	ros::spin();
	
	return 0;

}
