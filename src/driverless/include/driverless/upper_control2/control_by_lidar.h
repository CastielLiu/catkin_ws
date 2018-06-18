#ifndef CONTROL_BY_LIDAR_H_
#define CONTROL_BY_LIDAR_H_
#include<ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include<visualization_msgs/Marker.h>

#include<iostream>
#define TARGET_NUM 30
#define POINT_NUM_CYCLE 576
#define CLUSTER_MIN_DIS 0.17 //17cm
#define TARGET_DIS_SCOPE  3.0 //3m

#define SHOW_TARGET 1

typedef struct 
{
	float angle;
	float distance;
}polar_point_t;

typedef struct
{
	polar_point_t start_point;
	polar_point_t middle_point;
	polar_point_t end_point;
}targetMsg;


class Control_by_lidar
{
	private:
		void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
		char whereBarrier(const sensor_msgs::LaserScan::ConstPtr& msg);
		ros::Subscriber lidar_sub;
		float CAR_FRONT_SAFETY_DIS;
		float CAR_LR_SAFETY_DIS;
		float ANGLE_BOUNDARY;
		targetMsg target[TARGET_NUM]; 
		unsigned char new_target_flag;
		polar_point_t last_valid_point;
		polar_point_t now_point;
#if(SHOW_TARGET==1)
		
		void write_marker(targetMsg * target);
		ros::Publisher target_pub;
#endif	
		float polar_p2p_dis2(polar_point_t point1,polar_point_t point2);
		void create_target(const sensor_msgs::LaserScan::ConstPtr& msg);
	public:
		geometry_msgs::Twist controlMsg;
		char IS_Barrier;
		Control_by_lidar();
		//~Control_by_lidar();
		void run();
		
};
#endif
