#include"control_by_gps.h"

void Control_by_gps::run()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	private_nh.param<float>("linear_speed",linear_speed_temp_buf,0.5);
	private_nh.param<float>("DisThreshold",DisThreshold,2.);
	private_nh.param<float>("RadiusThreshold",RadiusThreshold,1.);
	
	private_nh.param<float>("angular_speed_pid_Kp",angular_speed_pid.Kp,0.0);
	private_nh.param<float>("angular_speed_pid_Ki",angular_speed_pid.Ki,0.0);
	private_nh.param<float>("angular_speed_pid_Kd",angular_speed_pid.Kd,0.0);
	private_nh.param<std::string>("file_path",file_path,std::string("/home/ubuntu/projects/catkin_ws/src/driverless/data/1.txt"));
	
	fp = fopen(file_path.c_str(),"r");
	debug_fp = fopen("/home/wendao/projects/catkin_ws/src/driverless/data/debug.txt","w"); //use to record debug msg
	if(fp==NULL) 
	{
		ROS_INFO("open %s failed !!!",file_path.c_str());
		exit(0) ;
	}
	gps_sub = nh.subscribe<driverless::Gps>("/gps_data",5, &Control_by_gps::gps_callback,this);
	//ros::spin();
}

Control_by_gps::Control_by_gps()
{
	now_location   = {0.0,0.0,0.0};
	start_location = {0.0,0.0,0.0};
	target_location= {0.0,0.0,0.0};
	rectangular =    {0.0,0.0,0.0,0.0};//x,y,dis,yaw
	//初始化为1 "已经到达（第一个）目标点"，目的是把当前位置设置为起始点
	arrive_target_flag =1 ;
	
	PID_init(&angular_speed_pid);
}

Control_by_gps::~Control_by_gps()
{
	fclose(fp);
	fclose(debug_fp);
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

	static float steer_radius[2];
	static unsigned char i=0;
	if(i<10) i++;
	
	now_location.lon = gps_msg->lon;
	now_location.lat = gps_msg->lat; 
	now_location.yaw = gps_msg->yaw; 
	
	if(arrive_target_flag==1)
	{
	//离目标点距离小于阈值,读入一个新的目标点
		if(feof(fp))
		{
			ROS_INFO("finish ......");
			//ros::shutdown();
			exit(0);
		}
		fscanf(fp,"%lf,%lf\n",&target_location.lon,&target_location.lat); //read a new target point
		
		arrive_target_flag =0;  
	}
	
	relative_X_Y_dis_yaw(now_location,target_location,&rectangular,1); //当前点与目标点的相对信息
	
	t_yaw_now = rectangular.t_yaw ;
	
	dis2end = rectangular.distance;
	
	
	if(dis2end < DisThreshold) //arrive_target  prepare to read next point
		arrive_target_flag =1;
	
	//float lateral_err =  LateralError(t_yaw_start,t_yaw_now, dis2end);
	
	float yaw_err = now_location.yaw - t_yaw_now; 
	
	steer_radius[1] = 0.5*dis2end/sin(yaw_err*PI_/180) ; //modify DisThreshold -> dis2end
	fprintf(debug_fp,"%f\r\n",steer_radius[1]);
	
	float test_threshold = 1.0;
	
	if(i>5)//not the first time in the function  
	{		//刚进入程序时两者差别很大，由初值引起的，因此待程序稳定后再使用差值判断
		if((steer_radius[1] - steer_radius[0]) > test_threshold)  
			steer_radius[1] = steer_radius[0] + test_threshold;
		else if((steer_radius[1] - steer_radius[0]) < -test_threshold)
			steer_radius[1] = steer_radius[0] - test_threshold;
	}
		

	if(fabs(steer_radius[1]) < RadiusThreshold)
		linear_speed = 0.2;//转弯半径太小， 降低速度 
	else
		linear_speed = linear_speed_temp_buf;
		
	angular_speed = linear_speed / steer_radius[1] ;  
	
	steer_radius[0] = steer_radius[1];
	//angular_speed = PID1_realize(&angular_speed_pid,0.0,yaw_err);  //lateral_err -> yaw_err
		
	//printf("t_yaw_now=%f yaw= %f  yaw_err=%f  angular_speed=%f\r\n",t_yaw_now,now_location.yaw,yaw_err,angular_speed);
	
	//printf("dis2end = %f  lateral_err=%f turning_radius=%f\r\n\r\n",dis2end,lateral_err,turning_radius);
	//printf("gps_yaw=%f  dis2end=%f\n\r\n",gps_msg->yaw,dis2end);
	//printf("cal_speed = %f   cal_yaw =%f\n",speed,cal_yaw);
	
	controlMsg.angular.z = angular_speed;   
	controlMsg.linear.x = linear_speed;
	
	//control_pub.publish(controlMsg); //gps control the car have condition (are there a obstacle?)  
}
