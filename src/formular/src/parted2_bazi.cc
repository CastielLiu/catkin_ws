#include "formular/formular.h"
PointCloud space_part_height(PointCloud cloud)
{
	sensor_msgs::PointCloud2 output;
	PointCloud cloud_filtered;
	//std::cout<<cloud.points.size()<<std::endl;
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator it;
	for(it = cloud.points.begin(); it != cloud.points.end(); it++)
	{	
		
		if(it->z > -0.2){
			cloud_filtered.points.push_back (*it);
			//std::cout<<it->x<<"\t"<<it->y<<"\t"<<it->z<<std::endl;
		}
		
		
	}
	cloud_filtered.header = cloud.header;
	cloud_filtered.width = cloud_filtered.points.size ();
  	cloud_filtered.height = 1;
  	cloud_filtered.is_dense = false;
	//std::cout<<cloud_filtered.points.size()<<std::endl;
	
	return cloud_filtered;
}

class cloudHandler
{
public:
	cloudHandler()
	{
		sub = n.subscribe("parted_points", 10,  &cloudHandler::cloud_cb, this);
		pubxyz = n.advertise<sensor_msgs::PointCloud2> ("height_points", 10);
	}
	
	void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
	{
		PointCloud cloud_init;
		pcl::fromROSMsg(cloud_msg, cloud_init);
		
		PointCloud cloud_parted;
		cloud_parted = space_part_height(cloud_init);
	
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
	ros::init (argc, argv, "height");
	cloudHandler handler;
	ros::spin();
	return 0;
}
