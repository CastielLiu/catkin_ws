#include"ros/ros.h"
#include"driverless/Gps.h"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"ReadGpsPointAndPublish_node");
	ros::NodeHandle nh;
	ros::Publisher gps_pub = nh.advertise<driverless::Gps>("gps_data",1);
	ros::Rate loop_rate(20);
	
	driverless::Gps gps;
	
	FILE *fp;
	
	char gpsDataFileName[] = "src/driverless/data/gps_point.txt";
	
	//system("pwd");
	
	fp = fopen(gpsDataFileName,"r");
	if(fp == NULL)
	{
		ROS_INFO("%s open failed",gpsDataFileName);
		ros::shutdown();
	}
	
	while(ros::ok())
	{
		if(!feof(fp))
			fscanf(fp,"%lf\t%lf\n",&gps.lon,&gps.lat);
		else
			return 0;
		gps.header.stamp = ros::Time::now();
		
		gps.header.frame_id = "gps_frame";
		
		gps_pub.publish(gps);
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	return 0;
}
