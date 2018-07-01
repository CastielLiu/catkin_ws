#include"control_by_gps.h"
#include"control_by_lidar.h"


int main(int argc,char**argv)
{
	ros::init(argc,argv,"upper_control_node");
	
	ros::NodeHandle nh;
	
	ros::NodeHandle private_nh("~");
	
	ros::Publisher control_pub;
	
	control_pub= nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	
	bool open_evade;
	
	private_nh.param<bool>("open_evade",open_evade,0);
	
	geometry_msgs::Twist controlMsg;
	
	Control_by_gps gps;
	
	Control_by_lidar lidar;
 	
 	gps.run();
 	
 	lidar.run();
 	
 	ros::Rate rate(20);
 	
 	while(ros::ok())
 	{
		if(open_evade ==1 && lidar.barrier_num !=0)
		{
			controlMsg = lidar.controlMsg;
		}
		else
		{
			controlMsg = gps.controlMsg;
 			//controlMsg.angular.z = 0; 
			//controlMsg.linear.x = 0.3;
		}

	 	//ROS_INFO("barrier_num = %d",lidar.barrier_num);	

 		control_pub.publish(controlMsg);
 		
 		ros::spinOnce();
 		
 		rate.sleep();
 	
 	}
 	return 0;
}


