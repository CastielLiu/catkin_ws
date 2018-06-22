#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/PointStamped.h>
#include<sensor_msgs/LaserScan.h>


void transformPoint(const tf::TransformListener& listener)
{
	//geometry_msgs::PointStamped laser_point;
	sensor_msgs::LaserScan laser_point;
	
	laser_point.angle_min = 0;
	laser_point.angle_max = 2*3.1415926;
	laser_point.angle_increment = 3.1415926/2;
	laser_point.scan_time = ros::Time();
	laser_point.range_min =0.2;
	laser_point.range_max =20;
	laser_point.ranges.resize(4);
	
	laser_point.ranges[0]=1;
	laser_point.ranges[1]=2.;
	laser_point.ranges[2]=3.;
	laser_point.ranges[3]=4.;
	
	
	laser_point.header.frame_id="lidar_frame";
	
	laser_point.header.stamp=ros::Time();
	
	//laser_point.point.x = 1.0;
	//laser_point.point.y =2.0;
	//laser_point.point.z = 0.0;
	
	//geometry_msgs::PointStamped base_point;
	sensor_msgs::LaserScan base_point;
	
	listener.transformPoint("base_link",laser_point,base_point);
	
	//ROS_INFO("base_laser:(%.2f, %.2f, %.2f)---->base_link:(%.2f, %.2f, %.2f) at time %.2f",
	//		laser_point.point.x,laser_point.point.y,laser_point.point.z,
	//		base_point.point.x,base_point.point.y,base_point.point.z,
	//		base_point.header.stamp.toSec());
			
	ROS_INFO("base_laser:(%.2f, %.2f, %.2f)---->base_link:(%.2f, %.2f, %.2f) at time %.2f",
			laser_point.ranges[0],laser_point.ranges[1],laser_point.ranges[2],
			base_point.ranges[0],base_point.ranges[1],base_point.ranges[2],
			base_point.header.stamp.toSec());
}




int main(int argc ,char** argv)
{
	ros::init(argc,argv,"car_tf_listener");
	ros::NodeHandle n;
	
	tf::TransformListener listener(ros::Duration(10));
	ros::Timer timer=n.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(listener)));
	
	ros::spin();
}
