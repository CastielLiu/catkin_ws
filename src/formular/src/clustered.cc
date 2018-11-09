#include "formular/formular.h"
ros::Publisher pubxyz;
void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
{
	PointCloud cloud_parted;
	pcl::fromROSMsg(cloud_msg, cloud_parted);
	
	PointCloud cloud_center;
	cloud_center = center_cluster(cloud_parted, 0.2, 10, 2500);
	
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(cloud_center, output);
	pubxyz.publish(output);
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "clustered");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("parted_points", 10, cloud_cb);
	pubxyz = n.advertise<sensor_msgs::PointCloud2> ("cluster_points", 10);
	ros::spin();
}
