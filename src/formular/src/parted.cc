#include "formular/formular.h"
class cloudHandler
{
public:
	cloudHandler()
	{
		sub = n.subscribe("pandar_points", 10,  &cloudHandler::cloud_cb, this);
		pubxyz = n.advertise<sensor_msgs::PointCloud2> ("parted_points", 10);
	}
	
	void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
	{
		PointCloud cloud_init;
		pcl::fromROSMsg(cloud_msg, cloud_init);
		
		PointCloud cloud_parted;
		cloud_parted = space_part(cloud_init, 100.0);
	
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(cloud_parted, output);
		pubxyz.publish(output);
	}
	
	
protected:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pubxyz;
	std::vector<double> heights;
};

int main(int argc, char** argv)
{
	ros::init (argc, argv, "parted");
	cloudHandler handler;
	ros::spin();
	return 0;
}
