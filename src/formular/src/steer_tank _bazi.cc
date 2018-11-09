#include "formular/formular.h"
#include <geometry_msgs/Twist.h>
ros::Publisher pub_steer;
int count = 0;
int enable =0;

double steerCreator_bazi(PointCloud cloud)
{
	double x = 0;
	double y = 0;
	double z = 0;
	double center_x = 0;
	double center_y = 0;
	double disToNext = 0;
	int left = -1;
	int right = -1;
	double theta=0.0;
	int num = 0;
	int big_count = 0.;
	double big_x1 = 0.;
	double big_y1 = 0.;
	double big_x2 = 0.;
	double big_y2 = 0.;
	double big_dis1 = 0.;
	if(cloud.points.size()<2 || ((cloud.points[0].x*cloud.points[0].x +cloud.points[0].y*cloud.points[0].y) > 100)) return 10000.;
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator iter;
	for(iter = cloud.points.begin(); iter != cloud.points.end(); iter++){
		std::cout<<"x:"<<iter->x<<"\t"<<"y:"<<iter->y<<std::endl;
	}
	
  	for(int i = 0; i < cloud.points.size(); i ++){
  		x = cloud.points[i].x;
  		y = cloud.points[i].y;
  		z = cloud.points[i].z;
  		if(z > 0){
  			if(big_count == 0) {
  				num++;
  				big_x1 = x;
  				big_y1 = y;
  				big_count++;
  			}
  			else if(big_count == 1){
  				num++;
  				big_x2 = x;
  				big_y2 = y;
  				big_count++;
  			}
  			else big_count = 0;
  		}
  		if(x < 0.0){
  			if(right < 0 && (x*x + y*y) < 10.0) left = i;
  			if(right >= 0 && ((x*x + y*y) < 10.0)) left = i; 
  		}
  		else{
  			if(left < 0 && ((x*x + y*y) < 10.0) right = i;
  			if(left >= 0 && ((x*x + y*y) < 10.0)) right = i; 
  		}
  		if(left >=0 && right >= 0){
	  		center_x = (cloud.points[left].x+cloud.points[right].x)/2.0;
	  		center_y = (cloud.points[left].y+cloud.points[right].y)/2.0;
			std::cout<<"left:"<<left<<" right:"<<right<<"("<<center_x<<","<<center_y<<")"<<std::endl;
			disToNext = sqrt(center_x*center_x + center_y*center_y);
			if(disToNext > 1.0){
				theta = (atan2(-1.0, 0) - atan2(center_y, center_x))/M_PI*180.0;	
				break;
			}
			else{
				left = -1;
				right = -1;
			}
		}
		
	}
	if(num != 0) enable = 1;
	if(enable == 1 && num == 0) {
		count++;
		enable = 0;
	}
	if(num == 1 && count == 0) {
		big_dis1 = big_x1 * big_x1 + big_y1 * big_y1;
		if(big_dis1 > 1.0) theta = 0;
		else theta = 45;
	}
	if(num == 2 && count == 2){
		center_x = (big_x1 + big_x2)/2.;
		center_y = (big_y1 + big_y2)/2.;
		theta = (atan2(-1.0, 0) - atan2(center_y, center_x))/M_PI*180.0;
	}
	std::cout<<"count"<<count<<" num"<<num<<":"<<theta<<"\tdisToNext:"<<disToNext<<std::endl;
	return theta;
}
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
	
	double steer_angle = steerCreator_bazi(cloud_center);
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
