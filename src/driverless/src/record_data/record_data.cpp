#include<ros/ros.h>
#include<driverless/Gps.h>
#ifndef PI_
#define PI_ 3.141592653589
#endif


class Record
{
	private:
		void gps_callback(const driverless::Gps::ConstPtr& gpsMsg);
		FILE *fp;
		driverless::Gps last_point;
		float sample_distance;
		
	public:
		Record();
		~Record();	
		void run();	
};

Record::Record()
{
	last_point.lon = 0.0;
	last_point.lat = 0.0;
}

Record::~Record()
{
	fclose(fp);
}

void Record:: run()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	ros::Subscriber gps_sub;
	fp = fopen("/home/wendao/projects/catkin_ws/src/driverless/data/gps.txt","w");
	if(fp == NULL)
	{
		ROS_INFO("open record data failed !!!!!");
		return ;
	}
	
	private_nh.param<float>("sample_distance",sample_distance,0.1);
	
	gps_sub= nh.subscribe("/gps_data",1,&Record::gps_callback,this);
	
	ros::spin();
}


void Record::gps_callback(const driverless::Gps::ConstPtr& gpsMsg)
{
	float x = (gpsMsg->lon -last_point.lon)*111000*cos(gpsMsg->lat *PI_/180.);
	float y = (gpsMsg->lat -last_point.lat ) *111000;

	if(x*x+y*y >= sample_distance*sample_distance)	
	{
		fprintf(fp,"%.8f\t%.8f\r\n",gpsMsg->lon,gpsMsg->lat);
		ROS_INFO("%.8f\t%.8f\r\n",gpsMsg->lon,gpsMsg->lat);
		last_point = *gpsMsg;
	}

}


int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_data_node");
	
	Record record;
	record.run();
	
	//ros::Timer printMsg_timer = nh.createTimer(ros::Duration(1.0),printMsg_timer_callback);
	//ros::Rate rate(10);


	return 0;

}

