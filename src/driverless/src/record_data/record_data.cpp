#include<ros/ros.h>
#include<driverless/Gps.h>
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>
#include <std_msgs/Int8.h>

#ifndef PI_
#define PI_ 3.141592653589
#endif

class Record
{
	private:
		void gps_callback(const driverless::Gps& gpsMsg);
		void timerCallback(const ros::TimerEvent&);
		
		std::string file_path;
		FILE *fp;
		driverless::Gps last_point , current_point;
		float sample_distance;
		ros::Subscriber gps_sub;
		ros::Timer timer;
	public:
		Record();
		~Record();	
		void run();	
		void recordToFile();
		int mode;	
};

Record::Record()
{
	last_point.lon = 0.0;
	last_point.lat = 0.0;
	mode = 0;
}

Record::~Record()
{
	fclose(fp);
}

void Record:: run()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	private_nh.param<int>("record_mode",mode,0);
	
	
	private_nh.param<std::string>("file_path",file_path,"$(rospack find driverless)data/gps.txt");
	
	private_nh.param<float>("sample_distance",sample_distance,0.1);
    
	fp = fopen(file_path.c_str(),"w");
	
	if(fp == NULL)
	{
		ROS_INFO("open record data file failed !!!!!");
		ros::shutdown();
	}
	 
	gps_sub= nh.subscribe("/gps_data",1,&Record::gps_callback,this);
	//timer = nh.createTimer(ros::Duration(0.05),&Record::timerCallback,this);
}

void Record::recordToFile()
{
	fprintf(fp,"%.8f\t%.8f\r\n",current_point.lon,current_point.lat);
	ROS_INFO("%.8f\t%.8f\r\n",current_point.lon,current_point.lat);
}


//void Record::gps_callback(const driverless::Gps::ConstPtr& gpsMsg)
//{
//	float x = (gpsMsg->lon -last_point.lon)*111000*cos(gpsMsg->lat *PI_/180.);
//	float y = (gpsMsg->lat -last_point.lat ) *111000;

//	if(x*x+y*y >= sample_distance*sample_distance)	
//	{
//		fprintf(fp,"%.8f\t%.8f\r\n",gpsMsg->lon,gpsMsg->lat);
//		ROS_INFO("%.8f\t%.8f\r\n",gpsMsg->lon,gpsMsg->lat);
//		last_point = *gpsMsg;
//	}
//}

void Record::gps_callback(const driverless::Gps& gpsMsg)
{
	current_point.lat = gpsMsg.lat;
	current_point.lon = gpsMsg.lon;
		
	float x = (gpsMsg.lon -last_point.lon)*111000*cos(gpsMsg.lat *PI_/180.);
	float y = (gpsMsg.lat -last_point.lat ) *111000;

	//ROS_INFO("test  mode =%d",mode);	
	//ROS_INFO("distance=%f",sqrt(x*x+y*y));
	
	if(x*x+y*y >= sample_distance*sample_distance && mode==0)	
	{
		fprintf(fp,"%.8f\t%.8f\r\n",current_point.lon,current_point.lat);
		ROS_INFO("%.8f\t%.8f\r\n",current_point.lon,current_point.lat);
		last_point = current_point;
	}
}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_data_node");
	
	
	Record record;
	
	record.run();
	
	
	
	if(record.mode != 0)	
	{
		struct input_event key_event;
		
		char key_press_flag = 0;
			
		int keys_fd=open("/dev/input/event5",O_RDONLY|O_NONBLOCK);//只读&非阻塞MODE
		 
		if(keys_fd<=0)
		{   
		    ROS_INFO("open key_event file failed");
		    ros::shutdown();
		}   
		ros::Rate r(50);

		while(ros::ok())
		{
			read(keys_fd,&key_event,sizeof(struct input_event));
			//if(key_event.type!=0)
			//	printf("type==%i  key %i state %i \n",key_event.type,key_event.code,key_event.value);
			if(key_event.type==2 && key_event.code==8 && key_event.value ==-1 )
			{
				record.recordToFile();
			}
			//read(keys_fd,&key_event,sizeof(struct input_event));
		//	if(key_event.type==1 && key_event.code==272 && key_event.value==0) key_press_flag =0;
	
			ros::spinOnce();
			r.sleep();
		}
	}
	else
		ros::spin();
		

	return 0;

}
