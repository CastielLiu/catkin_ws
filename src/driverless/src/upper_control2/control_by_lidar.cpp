#include"control_by_lidar.h"
#include <sensor_msgs/LaserScan.h>
#ifndef PI_
#define PI_ 3.141592653589
#endif

#define FRONT_SAFETY_DIS 200  //cm
#define LR_SAFETY_DIS    40 //cm
#define ANGLE_BOUNDARY atan2(LR_SAFETY_DIS,FRONT_SAFETY_DIS)  //


char whereBarrier(sensor_msgs::LaserScan& msg)
{
	for(int i=0;i<sizeof(msg.ranges)/sizeof(msg.ranges[0]);i++)
	{
		float angle = msg.angle_increment * i;
		
		if(angle >0 && angle <ANGLE_BOUNDARY && 
		   msg.ranges[i] <FRONT_SAFETY_DIS && msg.ranges[i]>0)
		   return 1;
		else if(angle > ANGLE_BOUNDARY && angle < PI_/2 &&
				msg.ranges[i]<LR_SAFETY_DIS/sin(angle))
			return 2;
		else if(angle < 2*PI_ && angle > 2*PI_-ANGLE_BOUNDARY && 
		   msg.ranges[i] <FRONT_SAFETY_DIS && msg.ranges[i]>0)
		   return -1;
		else if(angle < 2*PI_ - ANGLE_BOUNDARY && angle > 3*PI_/2 &&
				msg.ranges[i]<LR_SAFETY_DIS/sin(2*PI_-angle))
			return -2;
		else
			return 0;  
	}
}


