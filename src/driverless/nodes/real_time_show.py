#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from driverless.msg import Gps

import matplotlib.pyplot as plt
import numpy as np
import commands

lontitude = 0
latitude = 0


def callback(msg):
	rospy.loginfo("%s,%s",msg.lon,msg.lat)
	global lontitude
	global latitude
	lontitude = msg.lon
	latitude = msg.lat
	
	
	
if __name__ =="__main__":
	
	rospy.init_node('Real_time_show',anonymous = True)
	
	rospy.Subscriber("gps_data",Gps,callback)
	
	plt.ion()
	
	plt.figure("Path Tracking")
	
	plt.xlim([31.,32.])
	plt.ylim([0,100])
	plt.xlabel("lontitude",size=20)
	plt.ylabel("latitude",size=20,color="blue")
	
	first_time_flag =1
	
	while not rospy.is_shutdown():
		plt.plot(lontitude,latitude,'r.',label="raw_data");
		plt.show()
		if first_time_flag==1:
			plt.legend(loc=1)
			first_time_flag =0
		
		plt.pause(0.1)
	
	_,file_path = commands.getstatusoutput('rospack find driverless')
	file_path +="/data/"
	
	plt.savefig(file_path+"effect.pdf")	#save the figure

	rospy.spin()
