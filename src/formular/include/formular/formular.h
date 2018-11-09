#ifndef __FORMULAR_H_
#define __FORMULAR_H_

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#ifndef M_PI
#define M_PI 3.1415926
#endif
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;  
PointCloud space_part(PointCloud cloud, double x_distance, double y_distance, double z_distance);
PointCloud space_part(PointCloud cloud, double slope);
PointCloud outlier_filter(PointCloud cloud, int MeanK, double Thresh);
PointCloud center_cluster(PointCloud cloud, double Tolerance, int MinSize, int MaxSize);
double steerCreator(PointCloud cloud);
std::vector<double> zCreator(PointCloud cloud, double x_distance, double y_distance);
#endif
std::vector<double> zCreator(PointCloud cloud, double x_distance, double y_distance)
{
	//std::cout<<cloud.points.size()<<std::endl;
	std::vector<double> heightMax;
	std::vector<double> heightMin;
	for(int i = 0; i < 40; i++) heightMax.push_back(-1.0);
	for(int i = 0; i < 40; i++) heightMin.push_back(1.0);
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator it;
	for(it = cloud.points.begin(); it != cloud.points.end(); it++)
	{	
		
		if(it->x > (x_distance - 0.5) && it->x < (x_distance + 0.5)&& it->y < 0 && it->y > y_distance && it->z < 1 && it->x !=NAN && it->y  != NAN && it->z != NAN){
			int index = int(-1*it->y*2);
			if(it->y >0 ) index = 0;
			if(index > 80) continue;
			//std::cout<<index<<std::endl;
			/*
			std::cout<<it->y<<std::endl;
			std::cout<<index<<std::endl;
			std::cout<<"Max:"<<heightMax[index]<<std::endl;
			std::cout<<"Min:"<<heightMin[index]<<std::endl;
			*/
			if(it->z > heightMax[index]) heightMax[index] = it->z;
			if(it->z < heightMin[index]) heightMin[index] = it->z;
			
			//std::cout<<it->x<<"\t"<<it->y<<"\t"<<it->z<<std::endl;
		}
		if(heightMax[0]  < -0.5) heightMax[0] =  heightMax[1];
		for(int j = 0; j <20 ; j++){
			if((heightMax[j] - heightMin[j]) > 0.1) heightMax[j] = heightMin[j]+0.05;
			if(heightMax[j] < -0.5 && j != 0) heightMax[j] = heightMax[j-1] + 0.05;
		}
		
	}
	
	
	return heightMax;
}

PointCloud space_part(PointCloud cloud, double slope)///上坡，下坡保证有锥筒即可
{
	sensor_msgs::PointCloud2 output;
	PointCloud cloud_filtered;
	//std::cout<<cloud.points.size()<<std::endl;
	//std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator it;
	/*for(it = cloud.points.begin(); it != cloud.points.end(); it++)
	{	
		if((it->z*it->z/(it->y*it->y+it->x*it->x))>slope){///利用点云到原点的斜率，椎筒点云更大
			cloud_filtered.points.push_back (*it);
		}
		
	}*/
	
	for(int i = 1; i < cloud.points.size(); i++)
	{	
		double z = cloud.points[i].z;
		double y = cloud.points[i].y;
		double x = cloud.points[i].x;
		double z2 = cloud.points[i].z - cloud.points[i-1].z;
		double y2 = cloud.points[i].y - cloud.points[i-1].y;
		double x2 = cloud.points[i].x - cloud.points[i-1].x;
		if(x > -1.5 && x < 1.5 && y < 0 && y > -10 && ((z2*z2/(x2*x2+y2*y2))>slope || z > -0.5)&&  (x*x + y*y) > 0.3*0.3&& x !=NAN && y  != NAN && z != NAN ){///检测斜率，椎捅斜率比路面大得多//高度肯定高于地面的点云留
			cloud_filtered.points.push_back (cloud.points[i]);
		}
		
	}
	cloud_filtered.header = cloud.header;
	cloud_filtered.width = cloud_filtered.points.size ();
  	cloud_filtered.height = 1;
  	cloud_filtered.is_dense = false;
	//std::cout<<cloud_filtered.points.size()<<std::endl;
	
	return cloud_filtered;
}


PointCloud space_part(PointCloud cloud, double x_distance, double y_distance, std::vector<double> z_distance)
{
	sensor_msgs::PointCloud2 output;
	PointCloud cloud_filtered;
	//std::cout<<cloud.points.size()<<std::endl;
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator it;
	for(it = cloud.points.begin(); it != cloud.points.end(); it++)
	{	
		if(it->x > (-1.0 * x_distance) && it->x < x_distance && it->y < 0 && it->y > y_distance && it->z < 1 && it->x !=NAN && it->y  != NAN && it->z != NAN){
			int zzz = int(-1*it->y*2);
			if(it->y > 0) zzz = 0;
			if(zzz > 80) continue;
			//std::cout<<zzz<<std::endl;
			if(it->z > (z_distance[zzz] + 0.1)){
				cloud_filtered.points.push_back (*it);
				//std::cout<<it->x<<"\t"<<it->y<<"\t"<<it->z<<std::endl;
			}
		}
		
	}
	cloud_filtered.header = cloud.header;
	cloud_filtered.width = cloud_filtered.points.size ();
  	cloud_filtered.height = 1;
  	cloud_filtered.is_dense = false;
	//std::cout<<cloud_filtered.points.size()<<std::endl;
	
	return cloud_filtered;
}

PointCloud outlier_filter(PointCloud cloud, int MeanK, double Thresh)
{
	PointCloud cloud_filtered;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
	statFilter.setInputCloud(cloud.makeShared());
	statFilter.setMeanK(MeanK);
	statFilter.setStddevMulThresh(Thresh);
	statFilter.filter(cloud_filtered);
	return cloud_filtered;
}

PointCloud center_cluster(PointCloud cloud, double Tolerance, int MinSize, int MaxSize)
{
	//PointCloud cloud_cluster;//存储每个类
	PointCloud cloud_center;// 存储每个类的质心
	pcl::PointXYZ point_center;//存储质心
	
  	// 创建用于提取搜索方法的kdtree树对象
 	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new 		pcl::search::KdTree<pcl::PointXYZ>);
  	tree->setInputCloud (cloud.makeShared());

  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //欧式聚类对象
  	ec.setClusterTolerance (Tolerance);                     // 设置近邻搜索的搜索半径为0.2m
  	ec.setMinClusterSize (MinSize);                 //设置一个聚类需要的最少的点数目为5
  	ec.setMaxClusterSize (MaxSize);               //设置一个聚类需要的最大点数目为2500
  	ec.setSearchMethod (tree);                    //设置点云的搜索机制
  	ec.setInputCloud (cloud.makeShared());
  	ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
  	//迭代访问点云索引cluster_indices,直到分割处所有聚类
  	int j = 0;
  	int count = 0;
  	int count_z = 0;
  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  { 
  	//迭代容器中的点云的索引，并且分开每个点云的高度值
  	count = 0;
  	point_center.x = 0.;
  	point_center.y = 0.;
  	point_center.z = 0.;
  	std::cout<<"id:"<<j+1<<"\t";
  	if(it->indices.size() > 1000) continue;//将较大的物体，例如人排除掉
  	count_z = 0;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)	{
     //设置保存点云的属性问题
     
	count ++;
    //cloud_cluster.points.push_back (cloud.points[*pit]);
    
    point_center.x += cloud.points[*pit].x;
    point_center.y += cloud.points[*pit].y;
    if(cloud.points[*pit].z > -0.2 && count_z<5) count_z++;//5个较高的点，才判为大椎捅
    
    }
    if(count_z == 5) point_center.z = 1.0;
    else point_center.z = -1.0;
    ////////////////////////////////////////////////////////大椎捅的高度定z为1,小定为-1;
    point_center.x /=  (double)count;
    point_center.y /=  (double)count;
    cloud_center.points.push_back(point_center);
    
  	j++;
     
    std::cout<<"number:"<<count<<std::endl;
  }
   
  //cloud_cluster.header = cloud.header;
  cloud_center.header.frame_id = "pandar";
  cloud_center.width = cloud_center.points.size ();
  //std::cout<<"size:"<<cloud_cluster.points.size()<<std::endl;
  cloud_center.height = 1;
  cloud_center.is_dense = false;
  return cloud_center;
}

double steerCreator(PointCloud cloud)
{
	double x = 0;
	double y = 0;
	double z = 0;
	double center_x = 0;
	double center_y = 0;
	double disToNext = 0;
	int left = -1;
	int right = -1;
	if(cloud.points.size()<2 || ((cloud.points[0].x*cloud.points[0].x +cloud.points[0].y*cloud.points[0].y) > 100)) return 10000.;
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator iter;
	for(iter = cloud.points.begin(); iter != cloud.points.end(); iter++){
		std::cout<<"x:"<<iter->x<<"\t"<<"y:"<<iter->y<<std::endl;
	}
  	for(int i = 0; i < cloud.points.size(); i ++){
  		x = cloud.points[i].x;
  		y = cloud.points[i].y;
  		z = cloud.points[i].z;
  		if(x < 0.0){
  			if(right < 0 && (x*x + y*y) < 10.0) left = i;
  			if(right >= 0 && (x*x + y*y) < 10.0) left = i; //最近的两对椎捅距离
  		}
  		else{
  			if(left < 0 && (x*x + y*y) < 10.0) right = i;
  			if(left >= 0 && (x*x + y*y) < 10.0) right = i; 
  		}
  		if(left >=0 && right >= 0){
	  		center_x = (cloud.points[left].x+cloud.points[right].x)/2.0;
	  		center_y = (cloud.points[left].y+cloud.points[right].y)/2.0;
			std::cout<<"left:"<<left<<" right:"<<right<<"("<<center_x<<","<<center_y<<")"<<std::endl;
			disToNext = sqrt(center_x*center_x + center_y*center_y);
			if(disToNext > 1.0){
				double theta = (atan2(-1.0, 0) - atan2(center_y, center_x))/M_PI*180.0;	
				std::cout<<"left"<<left<<" right"<<right<<":"<<theta<<"\tdisToNext:"<<disToNext<<std::endl;
				return theta;
			}
			else{
				left = -1;
				right = -1;
			}
		}
		
	}
	return -1;	
}
