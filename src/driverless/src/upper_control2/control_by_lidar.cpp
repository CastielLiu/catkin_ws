#include"control_by_lidar.h"

#ifndef PI_
#define PI_ 3.141592653589
#endif

#define SPEEP 0.3
#define RADIUS 1.2

void Control_by_lidar::callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	create_target(msg);
	generate_control_msg();
	
#if(SHOW_TARGET==1)	
	write_marker(target);
#endif	

}


Control_by_lidar::Control_by_lidar()
{
	IS_Barrier = 0;
	barrier_num = 0;
	multil_barrier_flag =0;
	turning_flag =0;
	
	ELUDE_FRONT_DIS = 2.5; //m
	ELUDE_LR_DIS = 0.50; //m
	ELUDE_ANGLE_BOUNDARY = atan2(ELUDE_LR_DIS,ELUDE_FRONT_DIS);
	
	STOP_FRONT_DIS =0.5;
	STOP_LR_DIS = 0.36;
	STOP_ANGLE_BOUNDARY = atan2(STOP_LR_DIS,STOP_FRONT_DIS);
	
	
	
	memset(target,sizeof(polar_point_t)*TARGET_NUM ,0);
	new_target_flag =0;
	new_blank_area_flag = 0;
	
	//ROS_INFO("ELUDE_ANGLE_BOUNDARY = %f",ELUDE_ANGLE_BOUNDARY);
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

char Control_by_lidar::point_in_scope(polar_point_t point, unsigned char area_flag) //point是否在区域？
{//area_flag =1,避障区, area_flag=2,急停区
	char status;
	switch(area_flag)
	{
	case 1:
		if(point.angle >=0 && point.angle <ELUDE_ANGLE_BOUNDARY && 
		   point.distance < ELUDE_FRONT_DIS && point.distance>0)
		   	status = 1;
		else if(point.angle > ELUDE_ANGLE_BOUNDARY && point.angle < PI_/2 &&
				point.distance<ELUDE_LR_DIS/sin(point.angle))
			status = 2;
		else if(point.angle < 2*PI_ && point.angle > 2*PI_-ELUDE_ANGLE_BOUNDARY && 
		   point.distance <ELUDE_FRONT_DIS && point.distance>0)
		   status = -1;
		else if(point.angle < 2*PI_ - ELUDE_ANGLE_BOUNDARY && point.angle > 3*PI_/2 &&
				point.distance<ELUDE_LR_DIS/sin(2*PI_-point.angle))
			status = -2;
		else
			status = 0;
		break;
	case 2:
		if(point.angle >=0 && point.angle <STOP_ANGLE_BOUNDARY && 
		   point.distance < STOP_FRONT_DIS && point.distance>0)
		   	status = 5;
		else if(point.angle > STOP_ANGLE_BOUNDARY && point.angle < PI_/2 &&
				point.distance<STOP_LR_DIS/sin(point.angle))
			status = 5;
		else if(point.angle < 2*PI_ && point.angle > 2*PI_-STOP_ANGLE_BOUNDARY && 
		   point.distance <STOP_FRONT_DIS && point.distance>0)
		   status = 5;
		else if(point.angle < 2*PI_ - STOP_ANGLE_BOUNDARY && point.angle > 3*PI_/2 &&
				point.distance<STOP_LR_DIS/sin(2*PI_-point.angle))
			status = 5;
		else
			status = 0;
		break;
	default:
		ROS_INFO("area_flag err!!");
		exit(1);
		break;
	}
	return status;
}

//连接目标的起点和末点，判断线段上是否有点出现在区
//area_flag =1,避障区, area_flag=2,急停区
char Control_by_lidar::target_in_scope(targetMsg target,unsigned char area_flag)
{
	polar_point_t point;
	//直线 rho = k*theta +b;
	float k =(target.end_point.distance - target.start_point.distance)/(target.end_point.angle - target.start_point.angle);
	float b = target.start_point.distance - k* target.start_point.angle;
	for(float angle=target.start_point.angle;fabs(angle - target.end_point.angle)> PI_/360 ;angle += PI_/360)
	{
		if(angle >2* PI_) angle -= 2*PI_;
		point.angle = angle;
		point.distance= k*angle+b;
		if(point_in_scope(point,area_flag)!=0)//线段上存在一点在区
		{
			if(target.middle_point.angle>3*PI_/2)
				return -1;
			else
				return  1;
		}
	}
	return 0;
}

void Control_by_lidar::cal_barrier_num(void)
{
	barrier_num = 0;
	for(int i=0;i<target_num;i++)
	{
		if(target_in_scope(target[i],1)!=0)//避障区
		{
			barrier[barrier_num] = target[i];
			barrier_num++;
		}
	}
}

unsigned char Control_by_lidar::emergency_stop(void)
{
	for(int i=0;i<barrier_num;i++)
	{
		if(target_in_scope(barrier[i],2)!=0) //紧急停车区
			return 1;
	}
	return 0;
}

void Control_by_lidar::generate_control_msg(void)
{
	cal_barrier_num();

	if(emergency_stop()==1) //是否需要紧急停车
	{
 		controlMsg.angular.z = 0; 
		controlMsg.linear.x = 0.0;
ROS_INFO("emergency stop!!!");
		return ;		
	}

	switch(barrier_num)
	{
		case 0: // 没有障碍物
			turning_flag =0;
			break;
		case 1: //1个障碍物
			if(multil_barrier_flag==0)
			{
				if(barrier[0].middle_point.angle>3*PI_/2)
				{
					turning_flag = 1;
				}
				else
				{
					turning_flag =-1;
				}
			}
			else//上一时刻为多障碍物,不作出任何动作，维持之前的转向角
			{//上一时右转 且当前障碍物中点在左侧 
				if(turning_flag = 1 &&barrier[0].middle_point.angle>3*PI_/2)
					multil_barrier_flag =0;
				//上一时左转 且当前障碍物中点在右侧 
				else if(turning_flag = -1 && barrier[0].middle_point.angle<PI_/2)
					multil_barrier_flag =0;
			}
			break;
			
		default : //多个障碍物
			if(multil_barrier_flag==0)
			{
				float mid_angle = cal_middle_angle(barrier[0].start_point.angle,barrier[barrier_num-1].end_point.angle);
				if(mid_angle >3*PI_/2)
				{
					turning_flag =1	;
				}
				else
				{
					turning_flag =-1;	
				}
				multil_barrier_flag = 1;
			}
			else//上一时刻为多障碍物,不作出任何动作，维持之前的转向角
			{//上一时右转 且当前障碍物中点在左侧 
				if(turning_flag = 1 &&barrier[barrier_num-1].middle_point.angle>3*PI_/2)
					multil_barrier_flag =0;
				//上一时左转 且当前障碍物中点在右侧 
				else if(turning_flag = -1 && barrier[0].middle_point.angle<PI_/2)
					multil_barrier_flag =0;
			}

			break;
	}
	if(turning_flag==1)
	{
		controlMsg.angular.z = -SPEEP/RADIUS;  //右转 
		controlMsg.linear.x = SPEEP;	
	}
	else if(turning_flag==-1)
	{
		controlMsg.angular.z = SPEEP/RADIUS;  //left转 
		controlMsg.linear.x = SPEEP;
	}
}



float Control_by_lidar::polar_p2p_dis2(polar_point_t point1,polar_point_t point2) //用于聚合障碍物
{
	float theta = point2.angle - point1.angle;
	
	return     point1.distance * point1.distance 
			 + point2.distance * point2.distance 
			 - 2*point1.distance*point2.distance*cos(theta);
}

float Control_by_lidar::p2p_projective_dis(polar_point_t point1,polar_point_t point2)//投影距离,用于计算可通行区域
{
	return point2.distance*sin(point2.angle) - point1.distance*sin(point1.angle);
}

float Control_by_lidar::cal_middle_angle(float angle1 , float angle2)
{
	float sum_angle=angle1 + angle2 ;
	if(angle1>3*PI_/2 && angle1<2*PI_ && //angle1 in left
		angle2 <PI_/2)                   //angle2 in right
		sum_angle += 2*PI_;
	if(sum_angle/2 >2*PI_)
		return sum_angle/2 -2*PI_;
	else
		return sum_angle/2;		
}

void Control_by_lidar::create_target(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	unsigned char target_seq =0;
	new_target_flag =0;
	memset(target,sizeof(polar_point_t)*TARGET_NUM ,0x00);//清空目标
	//unsigned char blank_area_seq =0;
	
	for(int i=0;i<POINT_NUM_CYCLE;i++)
	{
		now_point.angle = msg->angle_increment * (i+1) + 3*PI_/2;
		if(now_point.angle >=2*PI_) now_point.angle -= 2*PI_;

		now_point.distance = msg->ranges[i];
		
		if((now_point.angle>4*PI_/9 && now_point.angle<14*PI_/9 )//只求270--360 0--90度内的障碍物
			|| now_point.distance >TARGET_DIS_SCOPE) // 防止距离为inf时点记录到障碍物内部  //Invalid point
		{//在270度时，距离索引值为0，即起点，当角度不在求解范围之内时，continue，但若前一时刻有为完成的目标记录，
		//即new_target_flag=1，应将上一个有效点记录为目标末点
			if(new_target_flag==1) 
			{
				target[target_seq].end_point = last_valid_point;
				new_target_flag =0;  //!!!!
target[target_seq].middle_point.angle = cal_middle_angle(target[target_seq].start_point.angle , target[target_seq].end_point.angle);
															
					target[target_seq].middle_point.distance = (target[target_seq].start_point.distance
															+target[target_seq].end_point.distance)/2;
//printf("%d  %f,%f\t%f,%f\t,%f,%f\r\n",target_seq,target[target_seq].start_point.angle*180/PI_,
//									target[target_seq].start_point.distance,
//									target[target_seq].middle_point.angle*180/PI_,target[target_seq].middle_point.distance,
//									target[target_seq].end_point.angle*180/PI_,target[target_seq].end_point.distance);
				target_seq ++;

			}
			continue;
		}

		//ROS_INFO("------angle=%f\tdis=%f -------",now_point.angle*180/PI_,now_point.distance);
		if(now_point.distance > 0.05 && now_point.distance < TARGET_DIS_SCOPE) //valid target point 
		{
			if(new_target_flag==0) //a new target
			{
				new_target_flag =1; 
				target[target_seq].start_point = now_point;
				last_valid_point = now_point;
				
				//ROS_INFO("i=%d  %d--------.angle= %f",i,target_seq,now_point.angle*180/PI_);

			}
			else
			{
				//ROS_INFO("last_valid_point.angle= %f",last_valid_point.angle*180/PI_);
				
				float p2p_dis2 = polar_p2p_dis2(last_valid_point,now_point);
				if(p2p_dis2 < CLUSTER_MAX_DIS*CLUSTER_MAX_DIS)
				{
					last_valid_point = now_point;
				}
				else // a target create complete
				{
					new_target_flag =0;
					target[target_seq].end_point = last_valid_point;
					
					target[target_seq].middle_point.angle = cal_middle_angle(target[target_seq].start_point.angle , target[target_seq].end_point.angle);
															
					target[target_seq].middle_point.distance = (target[target_seq].start_point.distance
															+target[target_seq].end_point.distance)/2;
															
//					printf("%d  %f,%f\t%f,%f\t,%f,%f\r\n",target_seq,target[target_seq].start_point.angle*180/PI_,
//														  target[target_seq].start_point.distance,
//														target[target_seq].middle_point.angle*180/PI_,target[target_seq].middle_point.distance,
//														target[target_seq].end_point.angle*180/PI_,target[target_seq].end_point.distance);
					target_seq ++;
				}
			}	
		}
		//ROS_INFO("i = %d",i);
	}
	target_num = target_seq;
	//ROS_INFO("out____________cycle");
}

#if(SHOW_TARGET==1)
void Control_by_lidar::write_marker(targetMsg * target)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();
	marker.ns = "points";
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 0;

	marker.id = 0;
	marker.type = visualization_msgs::Marker::POINTS;
	
	for(int i=0;i<target_num;i++)
	{
		//if(target[i].start_point.distance ==0 )break;
		
		
		marker.scale.x = 0.05; 
		marker.scale.y = 0.05;
		
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.color.a = 1.0;//transparency 透明度 
	
		geometry_msgs::Point p;
		p.x = target[i].start_point.distance * cos(target[i].start_point.angle);
		p.y = target[i].start_point.distance * sin(target[i].start_point.angle);
		p.z =0;
		marker.points.push_back(p); //start_point
		
		p.x = target[i].middle_point.distance * cos(target[i].middle_point.angle);
		p.y = target[i].middle_point.distance * sin(target[i].middle_point.angle);
		marker.points.push_back(p); //mid_point

		p.x = target[i].end_point.distance * cos(target[i].end_point.angle);
		p.y = target[i].end_point.distance * sin(target[i].end_point.angle);	
		marker.points.push_back(p); //end_point
		
	}
	target_pub.publish(marker);
}



#endif


