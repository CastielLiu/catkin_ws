#include "formular/formular.h"
#include <geometry_msgs/Twist.h>
ros::Publisher pub_steer;
void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
{
	PointCloud cloud_center;
	pcl::fromROSMsg(cloud_msg, cloud_center);
	
	geometry_msgs::Twist steer;
	steer.linear.x = 0.;
	steer.linear.y = 0.;
	steer.linear.z = 0.;
	steer.angular.x = 0.;
	steer.angular.y = 0.;
	steer.angular.z = 0.;
	
	double steer_angle = steerCreator(cloud_center);
	if(steer_angle > 999) steer.linear.x = 0;
	else {
		steer.linear.x = 0.3;
		if(steer_angle > 5.) steer.angular.z = 0.3;
		else if(steer_angle < -5.) steer.angular.z = -0.3;
		else steer.angular.z = 0.;
	}
	pub_steer.publish(steer);

}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "steer");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("cluster_points", 10, cloud_cb);
	pub_steer = n.advertise<geometry_msgs::Twist> ("cmd_vel", 10);
	ros::spin();
}
