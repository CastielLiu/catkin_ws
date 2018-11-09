#include"gps.h"
#include "open_usart.h"
#include "driverless/Gps.h"
#include "driverless/Encode.h"
#include "ros/ros.h"

#define pi 3.14159265
#define READ_NUM  256
#define CRC16_CCITT 0x1021
#define INITIAL_REMAINDER  0xFFFF  

using namespace std;

unsigned char buf[READ_NUM+105];

gps_data_t * gpsPtr = new gps_data_t;


int main(int argc,char **argv)
{
	int len , remaind_data_cnt;
	unsigned char *buf_rear = buf;
	unsigned char *buf_head = buf;
	
	ros::init(argc,argv,"gps_node");
	
	std::string gps_port_name;
	//ros::param::get("~gps_port_name",gps_port_name);
	ros::NodeHandle nh_private("~");
	nh_private.param<std::string>("gps_port_name",gps_port_name,"/dev/ttyUSB0");
	int fd = open_usart((char *)(gps_port_name.c_str()));
	if(fd == -1) 
	{
		ROS_INFO("open gps port %s failed !",(char *)(gps_port_name.c_str()));
		return 0;
	}
	ROS_INFO("open gps port %s ok!",(char *)(gps_port_name.c_str()));
	set_speed(fd,115200);
	set_Parity(fd,8,1,'N');

	
	ros::NodeHandle nh;
	
	ros::Publisher gps_pub = nh.advertise<driverless::Gps>("gps_data",1);
	
	ros::Rate loop_rate(50);
	
	driverless::Gps gps_data;
	
 	while(ros::ok())
	{
		gps_data.header.stamp = ros::Time::now();
		gps_data.header.frame_id = "gps_fream";
		len = read(fd,buf_rear,READ_NUM);
		//printf("len =%d\r\n",len);
		if(len >0)
			buf_rear += len;
			
		while(buf_rear-buf_head +1 >= 105-1)
		{
			//ROS_INFO("11");
			//if(*buf_head==0x14 && *(buf_head+1)==0x64 && 
			if(*(unsigned short *)buf_head == 0x6414 &&
				crcCompute(buf_head+4,100) == ((verify_width_t)*(buf_head+3)<< 8)+*(buf_head+2) )
			{
				gpsPtr = (gps_data_t *)buf_head;
				
				gps_data.lat = *(double *)gpsPtr->latitude;
				gps_data.lon = *(double *)gpsPtr->longtitude;
				gps_data.yaw = *(float *)gpsPtr->yaw;
				
				gps_data.north_velocity =*(float *)gpsPtr->north_velocity;
				gps_data.east_velocity =*(float *)gpsPtr->east_velocity;
				
				gps_data.lat *= 180/pi;
				gps_data.lon *= 180/pi;
				gps_data.yaw *= 180/pi;
			//	ROS_INFO("22");
				gps_pub.publish(gps_data);
				
				break;
			}
			buf_head ++ ;		
		}
		buf_head += 104;
		for( remaind_data_cnt=0;remaind_data_cnt<(buf_rear-buf_head);remaind_data_cnt++)
			buf[remaind_data_cnt]=buf_head[remaind_data_cnt];
			
		buf_head = buf ;
		buf_rear = buf+remaind_data_cnt;
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	close(fd);
	return 0;
}
 


verify_width_t crcCompute(unsigned char *buf,unsigned int nBytes)
{
	verify_width_t remainder = INITIAL_REMAINDER;
	unsigned int byte;
	for(int i=0;i<nBytes;i++)
	{
		byte = (remainder >> (8*sizeof(verify_width_t)-8) ^ buf[i] );
		remainder = crcTable[byte] ^ (remainder <<8);	
	}
	return remainder;
}

void crcTableGenerate(unsigned char *crcTable)
{
 	verify_width_t remainder;  
    verify_width_t i; 
    for(i=0;i<256;i++)
    {
    	remainder = i << (8*sizeof(verify_width_t)-8);
    	for(unsigned char bit=0;bit<8;bit++)
    	{
    		if(remainder & 0x8000) 
    			remainder = (remainder << 1) ^ CRC16_CCITT;
    		else
    			remainder <<=1;
		}
		crcTable[i] = remainder;
	}
}
	

