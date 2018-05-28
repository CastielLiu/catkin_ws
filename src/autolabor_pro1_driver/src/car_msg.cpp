#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Float32.h>
#include<std_msgs/Int32.h>
#include<nav_msgs/Odometry.h>


int remaining_battery=0;
float current=0.0, voltage =0.0 , odometer=0.0;

void odom_callback(const nav_msgs::Odometry& odomMsg)
{
	;
}

void current_callback(const std_msgs::Float32& currentMsg)
{
	current = currentMsg.data;
}


void voltage_callback(const std_msgs::Float32& voltageMsg)
{
	voltage = voltageMsg.data;
}

void battery_callback(const std_msgs::Int32& batteryMsg)
{
	remaining_battery = batteryMsg.data;
}

void printMsg_timer_callback(const ros::TimerEvent &)
{
	ROS_INFO("odometer = %.2f",odometer,current,voltage,remaining_battery);
	ROS_INFO("current = %.2f",current);
	ROS_INFO("voltage = %.2f",voltage);
	ROS_INFO("batteryRemaining = %d%%",remaining_battery);
	std::cout << std::endl; 
}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"car_msg");
	
	ros::NodeHandle nh;
	
	ros::Subscriber odom_sub= nh.subscribe("/wheel_odom",10,odom_callback);
	
	ros::Subscriber current_sub = nh.subscribe("/current",1,current_callback);
	
	ros::Subscriber voltage_sub = nh.subscribe("/voltage",1,voltage_callback);
	
	ros::Subscriber battery_sub = nh.subscribe("/remaining_battery",1,battery_callback);
	
	ros::Timer printMsg_timer = nh.createTimer(ros::Duration(1.0),printMsg_timer_callback);
	//ros::Rate rate(10);

	ros::spin();

	return 0;

}
