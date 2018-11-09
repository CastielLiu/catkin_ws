#include<ros/ros.h>
#include<stdio.h>
#include<iostream>
#include<unistd.h>
#include<fcntl.h>
#include <linux/input.h>
#include"key_event/KeyEvent.h"

int main(int argc,char **argv)
{
	int keys_fd;
	int long_press_flag = 2;
    struct input_event t;
    std::string key_port_;
      
    ros::init(argc,argv,"key_event");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    private_nh.param<std::string>("key_port",key_port_,"/dev/input/event1");
    
    keys_fd=open(key_port_.c_str(),O_RDONLY|O_NONBLOCK);//只读&非阻塞MODE
    if(keys_fd<=0)
    {   
        printf("error\n");
        return -1; 
    } 
    
    ros::Publisher pub = nh.advertise<key_event::KeyEvent>("key_event",1);
    ros::Rate r(100);
    
    key_event::KeyEvent key_event_msg;
    
    while(ros::ok())
    {
      read(keys_fd,&t,sizeof(struct input_event));
      if(t.type==EV_KEY && t.value==1)//键盘事件
      {
      	key_event_msg.code = t.code;
      	key_event_msg.value = t.value;
      	pub.publish(key_event_msg);
      	
      //	while(long_press_flag==2) 
      //	{
      //		read(keys_fd,&t,sizeof(struct input_event));
      	//	long_press_flag = t.value;
      	//}
      //	long_press_flag = 0;
      }
      r.sleep();
    	
    }
    return 0;
}
