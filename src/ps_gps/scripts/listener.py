#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
from ps_gps.msg import gps
import random
import matplotlib.animation as animation

x = []
y = []
count = 0

def ps_plot():
	fig, ax = plt.subplots()
	ax.plot(x, y,'r*')
	plt.ion()
	plt.pause(2)
	
	
def callback(data):
	rospy.loginfo(rospy.get_caller_id() + 'latitude %f', data.lat)
	global x
	global y
	y.append(data.lon + random.uniform(1,10))
	x.append(data.lat + random.uniform(1,10))
	global count
	count = count + 1
	# print(count)
	if count % 20 == 0:
		ps_plot()

	
	
	
	# plt.pause(0.01)
	# plt.close()
	
def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber('gps_serial', gps, callback)
	
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()
