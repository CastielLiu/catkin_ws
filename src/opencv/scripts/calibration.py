#!/usr/bin/python

# -*- coding:utf-8 -*-

import cv2
import numpy as np

if __name__ == "__main__":
	img_path = "4.png"
	cal = cv2.imread(img_path)
	
	cv2.imshow("img",cal)
	

	cal_gray = cv2.cvtColor(cal,cv2.COLOR_RGB2GRAY)
	cv2.imshow("img_gray",cal_gray)
	
	ret, corners = cv2.findChessboardCorners(cal_gray, (9, 6),None)
	if ret == True:
		cal = cv2.drawChessboardCorners(cal, (9, 6), corners, ret)
		cv2.imshow("line",cal)
	
	
	cv2.waitKey()

