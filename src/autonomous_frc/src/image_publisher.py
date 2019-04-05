#!/usr/bin/env python
import cv2 as cv
import numpy as np
import rospy
import sys
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import  *
from sensor_msgs.msg import *
from nav_msgs.msg import  *
from cv_bridge import CvBridge, CvBridgeError
from time import time
Camera1=True
Camera2=False
global ID1
ID1 = 0
global ID2
ID2 = 0
def getCamera(id=0, cam=1):
	try:
		success, frame=cv.VideoCapture(id).read()
		if(success):
			if cam==1:
				global ID1
				ID1=id
			else:
				global ID2
				ID2=id
			return cv.VideoCapture(id)
		else:
			return getCamera(id+1)

	except:
		return getCamera(id+1)

bridge = CvBridge()
rospy.init_node('image_pub')
np.set_printoptions(suppress=True)
rate = rospy.Rate(22)
if Camera1:
	cap1=getCamera(0,1)
	image_pub_a = rospy.Publisher('/camera/rgb'+str(ID1), Image, queue_size=10000000)
	starttime_1 = time()
# if Camera2:
# 	cap2=getCamera(0,2)
# 	image_pub_b = rospy.Publisher('/camera/rgb'+str(ID2), Image, queue_size=320000)
# 	starttime_2 = time()
while not rospy.is_shutdown():
	try:
		if Camera1:
			ret1, frame1 = cap1.read()
			# if(ret1):
			frame1 = cv.cvtColor(frame1,cv.COLOR_RGB2GRAY)
				# if (time() - starttime_1) >= 0.03:
				# 	starttime_1 =time()
			image_pub_a.publish(bridge.cv2_to_imgmsg(frame1, 'mono8'))
		if Camera2:
			try:
				ret2, frame2 = cap2.read()
				if(ret2):
					frame2 = cv.cvtColor(frame2,cv.COLOR_RGB2GRAY)
					if (time() - starttime_2) >= 0.064:
						starttime_2 =time()
						image_pub_b.publish(bridge.cv2_to_imgmsg(frame2, 'mono8'))
			except:pass
	except:
		pass
	rate.sleep()
