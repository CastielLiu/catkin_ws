#include"control_by_gps.h"

void Control_by_gps::run()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	private_nh.param<float>("linear_speed",linear_speed_temp_buf,0.5);
	private_nh.param<float>("DisThreshold",DisThreshold,2.);
	private_nh.param<float>("RadiusThreshold",RadiusThreshold,1.);
	private_nh.param<float>("path_tracking_resolution",path_tracking_resolution,0.2);//defalt 0.2m
	
	private_nh.param<float>("angular_speed_pid_Kp",angular_speed_pid.Kp,0.0);
	private_nh.param<float>("angular_speed_pid_Ki",angular_speed_pid.Ki,0.0);
	private_nh.param<float>("angular_speed_pid_Kd",angular_speed_pid.Kd,0.0);
	private_nh.param<std::string>("file_path",file_path,std::string("/home/ubuntu/projects/catkin_ws/src/driverless/data/1.txt"));
	
	fp = fopen(file_path.c_str(),"r");
	
	for(int i=0;i++;i<MAX_TARGET_NUM)
	{
		fscanf(fp,"%f\t%f\n",Arr_target_point[i].lon,Arr_target_point[i].lat);
		total_target_num = i+1;
		if(fp==NULL) break;
	}
	
	
	debug_fp = fopen("/home/ubuntu/projects/catkin_ws/src/driverless/data/debug.txt","w"); //use to record debug msg
	if(debug_fp == NULL)
	{
		ROS_INFO("open %s failed !!!","debug.txt");
		exit(0);
	}
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
	new_target_flag = 1; //init new_target_flag = 1 
	current_target_seq = 0; // current_target_seq will ++;
	current_segment_seq =1;
	
	PID_init(&angular_speed_pid);
}

Control_by_gps::~Control_by_gps()
{
	fclose(fp);
	fclose(debug_fp);
}

float Control_by_gps::relative_dis_yaw(gpsMsg_t  gps_base,gpsMsg_t  gps,unsigned char disORyaw)
{
	x = (gps.lon -gps_base.lon)*111000*cos(gps.lat *PI_/180.);
	y = (gps.lat -gps_base.lat ) *111000;
	if(disORyaw = CAL_DIS)
		return  sqrt( x * x + y * y);
	else
		return  atan2( x, y)*180/PI_;
}


float Control_by_gps::LateralError(double t_yaw_start,double t_yaw_now,float dis2end)
{
	return (sin((t_yaw_start - t_yaw_now)*PI_/180)) *dis2end;
}

void Control_by_gps::gps_callback(const driverless::Gps::ConstPtr& gps_msg)   //Callback
{

/////////////////////////////////
	if(new_target_flag ==1)
	{
		new_target_flag =0; //reset
		
		current_target_seq ++;
		
		if(current_target_seq > total_target_num)
			current_target_seq =1;
		
		current_segment_seq = 1;
		
		current_target_point = Arr_target_point[current_target_seq - 1];
		
		if(current_target_seq == 0)
			last_target_point = Arr_target_point[total_target_num-1];
		else
			last_target_point = Arr_target_point[current_target_seq-2];
			
		dis_between_2_target = relative_dis_yaw(current_target_point,last_target_point,CAL_DIS); //the distance between two target
		total_segment_num = dis_between_2_target/path_tracking_resolution;
		lat_increment = (current_target_point.lat - last_target_point.lat)/total_segment_num;
		lon_increment = (current_target_point.lon - last_target_point.lon)/total_segment_num;
	}
	
	current_track_point.lon = last_target_point.lon + lon_increment*current_segment_seq;
	current_track_point.lat = last_target_point.lat + lat_increment*current_segment_seq;
////////////////////////////////////	
	


	now_location.lon = gps_msg->lon;
	now_location.lat = gps_msg->lat; 
	now_location.yaw = gps_msg->yaw; 
	
	
	//ROS_INFO("A_lon=%f\tA_lat=%f\tB_lon=%f\tB_lat=%f\r\n",now_location.lon,now_location.lat,target_location.lon,target_location.lat);
	
	dis2tracking_point = relative_dis_yaw(now_location,current_track_point,CAL_DIS);  
	
	t_yaw_now = atan2( x, y)*180/PI_;
	
	
	if(dis2tracking_point < DisThreshold) 
		current_segment_seq ++;
	if(current_segment_seq > total_segment_num)
		new_target_flag = 1;//switch to next target 
	
	//float lateral_err =  LateralError(t_yaw_start,t_yaw_now, dis2end);
	
	float yaw_err = now_location.yaw - t_yaw_now; 
	
#ifdef  TEST_RADIUS	

	static float steer_radius[2];
	static unsigned char i=0;
	if(i<10) i++; 
	
	steer_radius[1] = 0.5*dis2tracking_point/sin(yaw_err*PI_/180) ; //modify DisThreshold -> dis2tracking_point
	
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
		linear_speed = 0.5;//转弯半径太小， 降低速度 
	else
		linear_speed = linear_speed_temp_buf;
		
	angular_speed = linear_speed / steer_radius[1] ;  
	
	steer_radius[0] = steer_radius[1];
#else
	float steer_radius = 0.5*dis2tracking_point/sin(yaw_err*PI_/180) ; 
	
	if(fabs(steer_radius) < RadiusThreshold)
		linear_speed = 0.5;//转弯半径太小， 降低速度 
	else
		linear_speed = linear_speed_temp_buf;
	
	angular_speed = linear_speed / steer_radius;  
#endif
	
	//angular_speed = PID1_realize(&angular_speed_pid,0.0,yaw_err);  //lateral_err -> yaw_err
		
	//printf("t_yaw_now=%f yaw= %f  yaw_err=%f  angular_speed=%f\r\n",t_yaw_now,now_location.yaw,yaw_err,angular_speed);
	
	//printf("dis2end = %f  lateral_err=%f turning_radius=%f\r\n\r\n",dis2end,lateral_err,turning_radius);
	//printf("gps_yaw=%f  dis2end=%f\n\r\n",gps_msg->yaw,dis2end);
	//printf("hhhhh\n");
	
	controlMsg.angular.z = angular_speed;   
	controlMsg.linear.x = linear_speed;
	
	ROS_INFO("t_yaw=%f\tyaw=%f\tyaw_err=%f\tdis2end=%f",t_yaw_now,now_location.yaw,yaw_err,dis2tracking_point);
	ROS_INFO("target:%d//%d\tsegment:%d//%d\r\n",current_target_seq,total_target_num,current_segment_seq,total_segment_num);
	
	//control_pub.publish(controlMsg); //gps control the car have condition (are there a obstacle?)  
}
