#include "formular/formular.h"
ros::Publisher pub_steer;
void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
{
	PointCloud cloud_center;
	pcl::fromROSMsg(cloud_msg, cloud_center);
	
	std_msgs::Float64 steer;
	steer.data = steerCreator(cloud_center);
	pub_steer.publish(steer);

}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "steer");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("cluster_points", 10, cloud_cb);
	pub_steer = n.advertise<std_msgs::Float64> ("lidar_steer", 10);
	ros::spin();
}
