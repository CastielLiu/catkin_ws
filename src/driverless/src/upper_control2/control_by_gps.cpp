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
	private_nh.param<std::string>("file_path",file_path,std::string("/home/wendao/projects/catkin_ws/src/driverless/data/1.txt"));
	
	fp = fopen(file_path.c_str(),"r");
	
	if(fp==NULL)
	{
		ROS_INFO("open file failed");
		exit(0);
	}
	
	debug_fp = fopen("/home/ubuntu/projects/catkin_ws/src/driverless/data/debug.txt","w");
	
	if(debug_fp==NULL)
	{
		ROS_INFO("open debug.txt failed");
		exit(0);
	}
	
	
#if TRACKING_MODE == VERTEX_TRACKING	
	for(int i=0;i<MAX_TARGET_NUM;i++)
	{
		fscanf(fp,"%lf\t%lf\n",&Arr_target_point[i].lon,&Arr_target_point[i].lat);
		total_target_num = i+1;
		if(feof(fp)) break;
	}
	ROS_INFO("total_target_num=%d",total_target_num);
	for(int i=0;i<total_target_num;i++)
		ROS_INFO("%f,%f",Arr_target_point[i].lon,Arr_target_point[i].lat);
#elif TRACKING_MODE == CURVE_TRACKING
	for(int i=0;i<5;i++)
	{
		fscanf(fp,"%lf\t%lf\n",&fit_point[i].lon,&fit_point[i].lat);
		if(feof(fp)) break;
	}
	
	 x1 = fit_point[0].lat - fit_point[0].lat;
	 y1 = fit_point[0].lon - fit_point[0].lon;
	
	 x2 = (fit_point[1].lat - fit_point[0].lat )*100000;
	 y2 = (fit_point[1].lon - fit_point[0].lon)*100000;
	
	 x3 = (fit_point[2].lat - fit_point[0].lat)*100000;
	 y3 = (fit_point[2].lon - fit_point[0].lon)*100000;
	
	 x4 = (fit_point[3].lat - fit_point[0].lat)*100000;
	 y4 = (fit_point[3].lon - fit_point[0].lon)*100000;
	 
	 std::cout << x1 <<"  "<<x2<<"  "<<x3<<"  "<<x4<<"  "<<y1<<"  "<<y2<<"  "<<y3<<"  "<<y4<<"  "<<std::endl;
	
	coefficient[3] = ((((y3-y2)/(x3-x2)-(y2-y1)/(x2-x1))/(x3-x1))-(((y4-y3)/(x4-x3)-(y3-y2)/(x3-x2))/(x4-x2)))/(x1-x4);
	coefficient[2] = ((y3-y1)/(x3-x1)-(y2-y1)/(x2-x1)-coefficient[3]*(x3*x3-x2*x2+x1*x3-x1*x2))/(x3-x2);
	coefficient[1] = ((y2-y1)-coefficient[2]*(x2-x1)*(x2+x1)-coefficient[3]*(x2*x2*x2-x1*x1*x1))/(x2-x1);
	coefficient[0] = y1-coefficient[1]*x1-coefficient[2]*x1*x1-coefficient[3]*x1*x1*x1;	
	for(int i =0;i<4;i++)
		printf("coefficient[i]%lf\r\n",coefficient[i]);

	///////点读入以后计算多项式系数 here
	total_segment_num = (fit_point[3].lat - fit_point[0].lat)/0.000001;
	current_segment_seq = 1;
#endif	
	
	//debug_fp = fopen("$(rospack find driverless)/data/debug.txt","w"); //use to record debug msg
	//if(debug_fp == NULL)
	//{
	//	ROS_INFO("open %s failed !!!","debug.txt");
	//	exit(0);
	//}
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
#if TRACKING_MODE == VERTEX_TRACKING
	new_target_flag = 1; //init new_target_flag = 1 
	current_target_seq = 0; // current_target_seq will ++;
#endif
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
	//ROS_INFO("%f\t%f\t%f\t%f",gps.lon,gps.lat,gps_base.lon,gps_base.lat);
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
	now_location.lon = gps_msg->lon;
	now_location.lat = gps_msg->lat; 
	now_location.yaw = gps_msg->yaw; 

#if TRACKING_MODE==VERTEX_TRACKING
	if(new_target_flag ==1)
	{
		new_target_flag =0; //reset
		
		current_target_seq ++;
		
		if(current_target_seq > total_target_num) //当现在的目标序号大于总的目标数时，说明目标已经跟踪了一个循环
			current_target_seq =1; //目标序号置1，重新跟踪第一个目标
		
		current_segment_seq = 1; //刚更新了目标序列，因此新目标跟踪子片段序列为1
		
		if(current_target_seq == 1) //当前目标序列为1，表明上一个目标为目标序列的最后一个
			last_target_point = Arr_target_point[total_target_num-1];
		else
			last_target_point = current_target_point;
			
		current_target_point = Arr_target_point[current_target_seq - 1];
		
		//ROS_INFO("current_target_point=%f\t%f",current_target_point.lat,current_target_point.lon);
		
			
		dis_between_2_target = relative_dis_yaw(current_target_point,last_target_point,CAL_DIS); //the distance between two target
		
		total_segment_num = dis_between_2_target/path_tracking_resolution;
		
	//	ROS_INFO("dis_between_2_target=%f\tpath_tracking_resolution=%f",dis_between_2_target,path_tracking_resolution);
		
		lat_increment = (current_target_point.lat - last_target_point.lat)/total_segment_num; //计算每个跟踪点的经纬度增量
		lon_increment = (current_target_point.lon - last_target_point.lon)/total_segment_num;
	}
	
	current_track_point.lon = last_target_point.lon + lon_increment*current_segment_seq;//计算当前跟踪点的经纬度
	current_track_point.lat = last_target_point.lat + lat_increment*current_segment_seq;
////////////////////////////////////	
	
	//ROS_INFO("A_lon=%f\tA_lat=%f\tB_lon=%f\tB_lat=%f\r\n",now_location.lon,now_location.lat,last_target_point.lon,last_target_point.lat);
	
	dis2tracking_point = relative_dis_yaw(now_location,current_track_point,CAL_DIS);  //计算当前点到当前跟踪点的距离
	
	t_yaw_now = atan2( x, y)*180/PI_;  //xy为类成员变量，用作记录两点的相对坐标，上面调用了relative__dis_yaw，计算了xy
	
	
	if(dis2tracking_point < DisThreshold) //当前点与当前跟踪点距离小于DisThreshold时，切换到下一个跟踪点
		current_segment_seq ++;
		
	ROS_INFO("dis2tracking_point = %f",dis2tracking_point);	
	if(current_segment_seq > total_segment_num)//当前跟踪点序号大于总跟踪片段时，表明当前目标点跟踪完毕，切换到下一个目标点
		new_target_flag = 1;//switch to next target 
		
	ROS_INFO("target:%d/%d\tsegment:%d/%d\r\n",current_target_seq,total_target_num,current_segment_seq,total_segment_num);
	
	//float lateral_err =  LateralError(t_yaw_start,t_yaw_now, dis2end);
	
#elif TRACKING_MODE == CURVE_TRACKING
	
	float temp1 = x1 + 0.1 * current_segment_seq;
	current_track_point.lat = temp1 * 0.00001 + fit_point[0].lat;
	
	float temp2 = coefficient[0] + coefficient[1] * temp1 +
							  coefficient[2]*temp1 * temp1 + 
							  coefficient[3]*temp1*temp1*temp1;
	current_track_point.lon = temp2 * 0.00001 + fit_point[0].lon;
							  
	dis2tracking_point = relative_dis_yaw(now_location,current_track_point,CAL_DIS);  //计算当前点到当前跟踪点的距离
	
	t_yaw_now = atan2( x, y)*180/PI_;  //xy为类成员变量，用作记录两点的相对坐标，上面调用了relative__dis_yaw，计算了xy
	
	
	if(dis2tracking_point < DisThreshold) //当前点与当前跟踪点距离小于DisThreshold时，切换到下一个跟踪点
		current_segment_seq ++;
	if(total_segment_num>current_segment_seq)	
		printf("%.7f\t%.7f\r\n",current_track_point.lon,current_track_point.lat);
	else
		ros::shutdown();

#endif	
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
	float steer_radius = 0.5*dis2tracking_point/sin(yaw_err*PI_/180) ; //预瞄准发计算期望转弯半径
	
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
	
	fprintf(debug_fp,"%.7f,%.7f,%.2f,%.7f,%.7f,%.2f,%.2f,%.2f\r\n",\
		now_location.lon,now_location.lat,now_location.yaw,current_track_point.lon,current_track_point.lat,t_yaw_now,angular_speed,linear_speed);
	
	//ROS_INFO("t_yaw=%f\tyaw=%f\tyaw_err=%f\tdis2end=%f",t_yaw_now,now_location.yaw,yaw_err,dis2tracking_point);
	
	
	//control_pub.publish(controlMsg); //gps control the car have condition (are there a obstacle?)  
}

