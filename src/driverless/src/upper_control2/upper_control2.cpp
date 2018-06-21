#include"control_by_gps.h"
#include"control_by_lidar.h"


int main(int argc,char**argv)
{
	ros::init(argc,argv,"upper_control_node");
	
	ros::NodeHandle nh;
	
	ros::Publisher control_pub;
	
	control_pub= nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	
	geometry_msgs::Twist controlMsg;
	
	Control_by_gps gps;
	
	Control_by_lidar lidar;
 	
 	gps.run();
 	
 	lidar.run();
 	
 	ros::Rate rate(20);
 	
 	while(ros::ok())
 	{
 		if(lidar.IS_Barrier ==0)
 		{
 			controlMsg = gps.controlMsg;
 			//controlMsg.angular.z = 0; 
			//controlMsg.linear.x = 0.1;
 		}
 			
 		else
 		{
	 		controlMsg = lidar.controlMsg;
	 		ROS_INFO("__IS_Barrier = %d",lidar.IS_Barrier);	
 		}
 	
 			
 		control_pub.publish(controlMsg);
 		
 		ros::spinOnce();
 		
 		rate.sleep();
 	
 	}
 	return 0;
}


