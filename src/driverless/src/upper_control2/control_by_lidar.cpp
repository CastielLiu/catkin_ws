#include"control_by_lidar.h"

#ifndef PI_
#define PI_ 3.141592653589
#endif

void Control_by_lidar::callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//ROS_INFO("lidar_callback");
	
#if(SHOW_TARGET==1)	
	create_target(msg);
	write_marker(target);

#endif	
	IS_Barrier = whereBarrier(msg);
	//ROS_INFO("IS_Barrier=%d",IS_Barrier);
	if(IS_Barrier<0)
	{
		controlMsg.angular.z = -0.1/1.;  //L R=3m
		controlMsg.linear.x = 0.1;
	}
	else if(IS_Barrier >0)
	{
		controlMsg.angular.z = 0.1/1.;  //R R=3m
		controlMsg.linear.x = 0.1;
	}
}

Control_by_lidar::Control_by_lidar()
{
	IS_Barrier = 0;
	
	CAR_FRONT_SAFETY_DIS = 2; //2m
	CAR_LR_SAFETY_DIS = 0.45; //0.4m
	
	ANGLE_BOUNDARY = atan2(CAR_LR_SAFETY_DIS,CAR_FRONT_SAFETY_DIS);
	memset(target,sizeof(polar_point_t)*TARGET_NUM ,0);
	new_target_flag =0;
	new_blank_area_flag = 0;
	
	//ROS_INFO("ANGLE_BOUNDARY = %f",ANGLE_BOUNDARY);
}

void Control_by_lidar::run()
{
	ros::NodeHandle nh;
	//ros::NodeHandle private_nh("~");
	
	lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1,&Control_by_lidar::callback,this);
#if (SHOW_TARGET==1)
	target_pub = nh.advertise<visualization_msgs::Marker>("show_target",1);
#endif
}

		
char Control_by_lidar::whereBarrier(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	for(int i=0;i<POINT_NUM_CYCLE;i++)
	{
		//ROS_INFO("%f",msg->ranges[i]);
		float angle = msg->angle_increment * (i+1);
		
		//printf("angle = %f\t dis= %f \r\n",angle,msg->ranges[i]);
		
		if(angle >0 && angle <ANGLE_BOUNDARY && 
		   msg->ranges[i] < CAR_FRONT_SAFETY_DIS && msg->ranges[i]>0)
		   return 1;
		else if(angle > ANGLE_BOUNDARY && angle < PI_/2 &&
				msg->ranges[i]<CAR_LR_SAFETY_DIS/sin(angle))
			return 2;
		else if(angle < 2*PI_ && angle > 2*PI_-ANGLE_BOUNDARY && 
		   msg->ranges[i] <CAR_FRONT_SAFETY_DIS && msg->ranges[i]>0)
		   return -1;
		else if(angle < 2*PI_ - ANGLE_BOUNDARY && angle > 3*PI_/2 &&
				msg->ranges[i]<CAR_LR_SAFETY_DIS/sin(2*PI_-angle))
			return -2;
	}
	return 0;
}

char Control_by_lidar target_in_scope(polar_point_t point)
{
	if(point.angle >0 && point.angle <ANGLE_BOUNDARY && 
	   point.distance < CAR_FRONT_SAFETY_DIS && point.distance>0)
	   return 1;
	else if(point.angle > ANGLE_BOUNDARY && point.angle < PI_/2 &&
			point.distance<CAR_LR_SAFETY_DIS/sin(point.angle))
		return 2;
	else if(point.angle < 2*PI_ && point.angle > 2*PI_-ANGLE_BOUNDARY && 
	   point.distance <CAR_FRONT_SAFETY_DIS && point.distance>0)
	   return -1;
	else if(point.angle < 2*PI_ - ANGLE_BOUNDARY && point.angle > 3*PI_/2 &&
			point.distance<CAR_LR_SAFETY_DIS/sin(2*PI_-point.angle))
		return -2;
	else
		return 0;
}

float Control_by_lidar::polar_p2p_dis2(polar_point_t point1,polar_point_t point2) //用于聚合障碍物
{
	float theta = point2.angle - point1.angle;
	
	return     point1.distance * point1.distance 
			 + point2.distance * point2.distance 
			 - 2*point1.distance*point2.distance*cos(theta);
}

float Control_by_lidar::p2p_projective_dis(polar_point_t point1,polar_point_t point2)//用于计算可通行区域
{
	return point2.distance*sin(point2.angle) - point1.distance*sin(point1.angle);
}

void Control_by_lidar::create_target(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	unsigned char target_seq =0;
	
	//unsigned char blank_area_seq =0;
	
	for(int i=0;i<POINT_NUM_CYCLE;i++)
	{
		now_point.angle = msg->angle_increment * (i+1);
		if(now_point.angle>PI_/2 && now_point.angle<3*PI_/2 ) continue; //只求0到180度内的障碍物
		
		now_point.distance = msg->ranges[i];
		if(now_point.distance==0 || now_point.distance>TARGET_DIS_SCOPE) //Invalid target point
			continue;
		else //valid target point 
		{
			if(new_target_flag==0) //a new target
			{
				new_target_flag =1; 
				target[target_seq].start_point = now_point;
				last_valid_point = now_point;
			}
			else
			{
				float p2p_dis2 = polar_p2p_dis2(last_valid_point,now_point);
				if(p2p_dis2 < CLUSTER_MAX_DIS*CLUSTER_MAX_DIS)
				{
					last_valid_point = now_point;
				}
				else // a target create complete
				{
					new_target_flag =0;
					target[target_seq].end_point = last_valid_point;
					target[target_seq].middle_point.angle = (target[target_seq].start_point.angle
															+target[target_seq].end_point.angle)/2;
															
					target[target_seq].middle_point.distance = (target[target_seq].start_point.distance
															+target[target_seq].end_point.distance)/2;
					target_seq ++;
				}
			}	
		}
	}
	//ROS_INFO("target_seq=%d",target_seq);
}

#if(SHOW_TARGET==1)
void Control_by_lidar::write_marker(targetMsg * target)
{
	visualization_msgs::Marker points;
	for(int i=0;i<TARGET_NUM;i++)
	{
		if(target[i].start_point.distance ==0 )break;
		
		points.header.frame_id = "/my_frame";
		points.header.stamp = ros::Time::now();
		points.ns = "points";
		points.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = 0;
	
		points.id = 0;
		points.type = visualization_msgs::Marker::POINTS;
		points.scale.x = 0.05;
		points.scale.y = 0.05;
		
		points.color.r = 0.0;
		points.color.g = 1.0;
		points.color.b = 0.0;
		points.color.a = 1.0;//transparency 透明度 
	
		geometry_msgs::Point p;
		p.x = target[i].start_point.distance * cos(target[i].start_point.angle);
		p.y = target[i].start_point.distance * sin(target[i].start_point.angle);
		p.z =0;
		
		points.points.push_back(p); //start_point
		
		points.color.r = 1.0;
		points.color.g = 0.0;
		points.color.b = 0.0;
		p.x = target[i].start_point.distance * cos(target[i].end_point.angle);
		p.y = target[i].start_point.distance * sin(target[i].end_point.angle);	
		points.points.push_back(p); //end_point
		
	}
	target_pub.publish(points);
}



#endif


