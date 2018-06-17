#include"control_by_lidar.h"

#ifndef PI_
#define PI_ 3.141592653589
#endif

void Control_by_lidar::callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//ROS_INFO("lidar_callback");
	create_target(msg);
#if(SHOW_TARGET==1)	
	write_marker(target);

#endif	
	char IS_Barrier = whereBarrier(msg);
	//ROS_INFO("IS_Barrier=%d",IS_Barrier);
	if(IS_Barrier<0)
	{
		controlMsg.angular.z = 0.5/3.;  //L R=3m
		controlMsg.linear.x = 0.5;
	}
	else if(IS_Barrier >0)
	{
		controlMsg.angular.z = -0.5/3.;  //R R=3m
		controlMsg.linear.x = 0.5;
	}
}

Control_by_lidar::Control_by_lidar()
{
	IS_Barrier = 0;
	CAR_FRONT_SAFETY_DIS = 2; //2m
	CAR_LR_SAFETY_DIS = 0.4; //0.4m
	ANGLE_BOUNDARY = atan2(CAR_LR_SAFETY_DIS,CAR_FRONT_SAFETY_DIS);
	memset(target,sizeof(polar_point_t)*TARGET_NUM ,0);
	new_target_flag =0;
	
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

float Control_by_lidar::polar_p2p_dis2(polar_point_t point1,polar_point_t point2)
{
	float theta = point2.angle - point1.angle;
	
	return     point1.distance * point1.distance 
			 + point2.distance * point2.distance 
			 - 2*point1.distance*point2.distance*cos(theta);
}

void Control_by_lidar::create_target(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	unsigned char target_seq =0;
	
	for(int i=0;i<POINT_NUM_CYCLE;i++)
	{
		now_point.angle = msg->angle_increment * (i+1);
		now_point.distance = msg->ranges[i];
		if(now_point.distance==0 || now_point.distance>20.0) //Invalid target point
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
				if(p2p_dis2 < CLUSTER_MIN_DIS*CLUSTER_MIN_DIS)
				{
					last_valid_point = now_point;
				}
				else // a target create complete
				{
					new_target_flag =0;
					target[target_seq].end_point = last_valid_point;
					target_seq ++;
				}
			}	
		}
	}
	ROS_INFO("target_seq=%d",target_seq);
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


