#include"upper_control2.h"


void Control_by_gps::run()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	private_nh.param<float>("linear_speed",linear_speed,0.5);
	private_nh.param<float>("DisThreshold",DisThreshold,2.);
	private_nh.param<float>("angular_speed_pid_Kp",angular_speed_pid.Kp,0.0);
	private_nh.param<float>("angular_speed_pid_Ki",angular_speed_pid.Ki,0.0);
	private_nh.param<float>("angular_speed_pid_Kd",angular_speed_pid.Kd,0.0);
	private_nh.param<std::string>("file_path",file_path,std::string("/home/wendao/catkin_ws2/src/driverless/data/data.txt"));
	
	fp = fopen(file_path.c_str(),"r");
	if(fp==NULL) 
	{
		ROS_INFO("open %s failed !!!",file_path.c_str());
		return ;
	}
	//fscanf(fp,"%lf,%lf\n",&target_location.lon,&target_location.lat); //read the first target point
	
	control_pub= nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	ros::Subscriber gps_sub = nh.subscribe<driverless::Gps>("/gps_data",1, &Control_by_gps::gps_callback,this);
	
	ros::spin();
}

Control_by_gps::Control_by_gps()
{
	now_location   = {0.0,0.0,0.0};
	start_location = {0.0,0.0,0.0};
	target_location= {0.0,0.0,0.0};
	rectangular =    {0.0,0.0,0.0,0.0};
	//初始化为1 "已经到达（第一个）目标点"，目的是把当前位置设置为起始点
	arrive_target_flag =1 ;
	
	PID_init(&angular_speed_pid);
}

Control_by_gps::~Control_by_gps()
{
	fclose(fp);
}

void Control_by_gps::relative_X_Y_dis_yaw(gps_sphere_t  gps_base,gps_sphere_t  gps,gps_rect_t *rectangular,unsigned char num)
{
	rectangular->x = (gps.lon -gps_base.lon)*111000*cos(gps.lat *PI_/180.);
	rectangular->y = (gps.lat -gps_base.lat ) *111000;
	if(num)
	{
		rectangular->distance = sqrt(rectangular->x * rectangular->x + rectangular->y * rectangular->y);
		rectangular->t_yaw = atan2(rectangular->x,rectangular->y)*180/PI_;
	}
}

float Control_by_gps::LateralError(double t_yaw_start,double t_yaw_now,float dis2end)
{
	return (sin((t_yaw_start - t_yaw_now)*PI_/180)) *dis2end;
}

void Control_by_gps::gps_callback(const driverless::Gps::ConstPtr& gps_msg)   //Callback
{
	now_location.lon = gps_msg->lon;
	now_location.lat = gps_msg->lat; 
	now_location.yaw = gps_msg->yaw;
	
	if(arrive_target_flag==1)
	{
	//如果到达了目标点，把当前点设为起始点，并且读入一个新的目标点
	//第一次回调时，把当前位置作为起始位置
		start_location = now_location;
		if(feof(fp))
		{
			ROS_INFO("finish ......");
			exit(0);
		}
		fscanf(fp,"%lf,%lf\n",&target_location.lon,&target_location.lat); //read a new target point
		
		relative_X_Y_dis_yaw(start_location,target_location,&rectangular,1); //计算起始期望航向角
		
		t_yaw_start = rectangular.t_yaw ;
		
		arrive_target_flag =0;  //!!!
	}
	
	
	relative_X_Y_dis_yaw(now_location,target_location,&rectangular,1); //当前点与目标点的相对信息
	
	t_yaw_now = rectangular.t_yaw ;
	
	dis2end = rectangular.distance;
	
	
	if(dis2end < DisThreshold)
		arrive_target_flag =1;
	
	float lateral_err =  LateralError(t_yaw_start,t_yaw_now, dis2end);
	
	float yaw_err = now_location.yaw - t_yaw_now; 
	
	angular_speed = PID1_realize(&angular_speed_pid,0.0,lateral_err);
	
	float speed = sqrt(gps_msg->east_velocity * gps_msg->east_velocity + gps_msg->north_velocity*gps_msg->north_velocity);
	float cal_yaw = atan2(gps_msg->east_velocity,gps_msg->north_velocity)*180./PI_;
	
	printf("yaw_err=%f\r\n",yaw_err);
	//printf("dis2end = %f  lateral_err=%f  angular_speed=%f\n",dis2end,lateral_err,angular_speed);
	//printf("east_velocity=%f  north_velocity=%f  gps_yaw=%f\n",gps_msg->east_velocity,gps_msg->north_velocity,gps_msg->yaw);
	//printf("cal_speed = %f   cal_yaw =%f\n",speed,cal_yaw);
	
	controlMsg.angular.z = angular_speed;   
	controlMsg.linear.x = linear_speed;
	
	control_pub.publish(controlMsg);  
}



int main(int argc,char**argv)
{
	ros::init(argc,argv,"upper_control_node");
	
	Control_by_gps gps;
 	
 	gps.run();
 	
 	return 0;
}


