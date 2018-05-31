#include"../include/ps_gps/serial_open.h"
#include"../include/ps_gps/ps_type.h"
#include"../include/ps_gps/ps_gps_func.h"


#include<ros/ros.h>
#include"ps_gps/gps.h"

#include<visualization_msgs/Marker.h>

#include<sstream>

const char *dev_name = "/dev/ttyUSB0";



int main(int argc, char **argv)
{
    int fd = 0, len = 0;
    
	if (argc == 1)
		fd = dev_open(dev_name);
	else if (argc == 2)
		fd = dev_open(argv[1]);
	else if (argc == 3)
		fd = dev_open(dev_name);
	else
		usage(argc, argv);
	
	ros::init(argc, argv, "ps_serial_driver");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<ps_gps::gps>("gps_serial_node", 1000);
	ros::NodeHandle n2;
	ros::Publisher chatter_pub2 = n2.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::Rate loop_rate(5);
	
	double time_now = ros::Time::now().toSec();
	
	unsigned char buf[READ_NUM];
	int count = 1;
	visualization_msgs::Marker points;
	ps_gps::gps GPS;
	while (ros::ok())
	{
						
		len = read(fd, buf, READ_NUM);
		if( len == -1)
			ROS_INFO("read %d bytes: error", len);
		else if(len < 105)
			ROS_INFO("len = %d", len);
		
		
		buf[len] = '\0';
		
		
		
		
		if (len ==  105) 
		{
			
			ps_anpp_gps_msg_t *gps_msg = (ps_anpp_gps_msg_t *)buf;
			
			gps_message_parse(GPS, gps_msg);
			
					
			chatter_pub.publish(GPS);
		}
		
		
		points.header.frame_id = "/my_frame";
		points.header.stamp = ros::Time::now();
		points.ns = "points";
		points.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = 0;
		
		points.id = 0;
		points.type = visualization_msgs::Marker::POINTS;
		points.scale.x = 1.2;
		points.scale.y = 1.2;
		
		points.color.g = 1.0;
		points.color.a = 1.0;
		
		geometry_msgs::Point p;
		p.x = (GPS.lat + count)/10;
		p.y = (GPS.lon + count)/10;
		p.z = 0;
		
		points.points.push_back(p);
		
		chatter_pub2.publish(points);			
		
		ros::spinOnce();
		
		loop_rate.sleep();
		++count;
		
		
	}
	
	
	return 0;

}
