#ifndef CONTROL_BY_GPS_H_
#define CONTROL_BY_GPS_H_
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<driverless/Gps.h>
#include"pid.h"
#include<iostream>

#ifndef PI_
#define PI_ 3.141592653589
#endif

typedef struct
{
	double lon;
	double lat;
	double yaw;
} gps_sphere_t;

typedef struct 
{
	double x;
	double y;
	double distance;
	double t_yaw;
}gps_rect_t;



class Control_by_gps
{
	private:
		gps_sphere_t now_location ,start_location ,target_location;
		gps_rect_t rectangular; //相对直角坐标信息
		//ros::Publisher control_pub;
		ros::Subscriber gps_sub ;
		
		FILE *fp;
		FILE *debug_fp;
		std::string file_path;
		double t_yaw_start,t_yaw_now;
		float dis2end;
		float DisThreshold,RadiusThreshold;
		unsigned char arrive_target_flag;
		
		float linear_speed,angular_speed , linear_speed_temp_buf;
		pid_t_ angular_speed_pid;
		pid_t_ liner_speed_pid;
		
		void init();
		float LateralError(double t_yaw_start,double t_yaw_now,float dis2end);
		void relative_X_Y_dis_yaw(gps_sphere_t  gps_base,gps_sphere_t  gps,gps_rect_t *rectangular,unsigned char num);
		
		
	public:
		geometry_msgs::Twist controlMsg;
		Control_by_gps();
		~Control_by_gps();
		void run();
		void gps_callback(const driverless::Gps::ConstPtr& gps_msg);
		
		
};



#endif
