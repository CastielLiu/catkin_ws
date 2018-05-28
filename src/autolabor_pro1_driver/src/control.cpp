#include<ros/ros.h>
#include<geometry_msgs/Twist.h>


int main(int argc,char**argv)
{
	ros::init(argc,argv,"control");
	
	ros::NodeHandle nh;
	
	ros::Publisher control_pub= nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	
	ros::Rate rate(10);
	
	float speed=0 , angle=0;
	
	ros::param::get("~speed",speed);
	
	ros::param::get("~angle",angle);
	
	geometry_msgs::Twist controlMsg;
	
	controlMsg.linear.x =speed;
	controlMsg.angular.z =angle;
	
	while(ros::ok())
	{
		control_pub.publish(controlMsg);
		
		ROS_INFO("sending.......\r\n");
		
		ros::spinOnce();
			
		rate.sleep();
	
	}
	
	return 0;

}
