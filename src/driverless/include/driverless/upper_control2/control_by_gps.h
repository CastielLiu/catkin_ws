#ifndef CONTROL_BY_GPS_H_
#define CONTROL_BY_GPS_H_
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<driverless/Gps.h>
#include"pid.h"
#include<iostream>

#define MAX_TARGET_NUM 4
#define CAL_DIS 1
#define CAL_YAW 0

#ifndef PI_
#define PI_ 3.141592653589
#endif

typedef struct
{
	double lon;
	double lat;
	double yaw;
} gpsMsg_t;




class Control_by_gps
{
	private:
		gpsMsg_t now_location ,start_location ,target_location;
		
		gpsMsg_t Arr_target_point[MAX_TARGET_NUM];//storage all target peak
		gpsMsg_t current_target_point ,last_target_point;
		gpsMsg_t current_track_point ;
		
		double lat_increment,lon_increment;
		int total_segment_num,current_segment_seq;
		unsigned char total_target_num,current_target_seq;
		float  dis_between_target;
		float  path_tracking_resolution;
		bool new_target_flag;
		
		float x,y; //减少系统计算时间，计算距离之后存储相对坐标，计算航向角时直接利用
		
	
		//ros::Publisher control_pub;
		ros::Subscriber gps_sub ;
		
		FILE *fp;
		FILE *debug_fp;
		std::string file_path;
		double t_yaw_start,t_yaw_now;
		
		float dis2tracking_point;
		float dis_between_2_target;
		
		float DisThreshold,RadiusThreshold;
		
		float linear_speed,angular_speed , linear_speed_temp_buf;
		pid_t_ angular_speed_pid;
		pid_t_ liner_speed_pid;
		
		void init();
		float LateralError(double t_yaw_start,double t_yaw_now,float dis2end);
		float relative_dis_yaw(gpsMsg_t  gps_base,gpsMsg_t  gps,unsigned char disORyaw);
		
		
	public:
		geometry_msgs::Twist controlMsg;
		Control_by_gps();
		~Control_by_gps();
		void run();
		void gps_callback(const driverless::Gps::ConstPtr& gps_msg);
		
		
};



#endif
