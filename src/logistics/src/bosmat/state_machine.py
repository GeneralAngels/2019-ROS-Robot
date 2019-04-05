#!/usr/bin/env python


import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import  *
from sensor_msgs.msg import *
from nav_msgs.msg import  *
import numpy as np
import tf
import time
from math import *


class Autonumos():
	def __init__(self):
		self.index= 0
		self.is_on = False
		self.action_order = ["auto","auto","image"]
		self.odom_intriusic = Odometry()


class robot_state():
	def __init__(self):
		self.image_processing = False
		self.autonumos = Autonumos()
		self.autonumos.is_on = False
		self.teleop = False
		self.odom_intriusic = Odometry()
		self.offsets = [0,0,0]
		self.x = 0
		self.y = 0
	def status_callback(self,status):
		status = status.data
		# print(status)
		if status=='teleop':
			# print("teleop")
			self.teleop = True
			self.autonumos.is_on  = False
			self.image_processing = False
		elif status == 'fms':
			self.teleop = False
			if self.autonumos.action_order[self.autonumos.index] =='auto':
				self.autonumos.is_on  = True
				self.image_processing = False
			elif self.autonumos.action_order[self.autonumos.index] == 'image':
				self.autonumos.is_on  = False
				self.image_processing = True
		elif status == 'auto':
			self.teleop = False
			self.autonumos.is_on  = False
			self.image_processing = True
	def path_follower_status_callback(self,status):
		if self.autonumos.is_on and status == "done":
			self.autonumos.index+=1
	def odom_callback(self,odom):
		# print("odom")
		quaternion = odom.pose.pose.orientation
		self.my_heading = tf.transformations.euler_from_quaternion((quaternion.x,quaternion.y,quaternion.z,quaternion.w))
		self.distance = self.my_heading[0]
		self.my_heading = self.my_heading[2]
		self.my_point = odom.pose.pose.position
		if self.teleop or (self.autonumos.is_on and self.autonumos.action_order[self.autonumos.index] =='auto'):
			self.x = 0
			self.y=0
			self.offsets = [self.my_point.x,self.my_point.y,self.my_heading]
			# print("offset is set")
		# print(self.offsets)
		else:
			self.x +=self.distance*cos(self.my_heading-self.offsets[2])
			self.y +=self.distance*sin(self.my_heading-self.offsets[2])
			self.odom_quat=tf.transformations.quaternion_from_euler(0,0,self.my_heading-self.offsets[2])
			self.odom_intriusic.pose.pose=Pose(Point(self.x,self.y,0),Quaternion(*self.odom_quat))

if __name__ == '__main__':
	rospy.init_node('state_machine')
	rate = rospy.Rate(50)
	state = robot_state()
	cmd_vel_sub = rospy.Subscriber('/robot_status', String, state.status_callback)
	cmd_vel_sub = rospy.Subscriber('/path_status', String, state.path_follower_status_callback)
	cmd_vel_sub = rospy.Subscriber('/odom', Odometry, state.odom_callback)
	odom_intriusic_publisher = rospy.Publisher('/odom_intriusic', Odometry, queue_size=20 )
	while not rospy.is_shutdown():
		rospy.set_param("teleop",state.teleop)
		rospy.set_param("autonumos",state.autonumos.is_on)
		rospy.set_param("image_processing",state.image_processing)
		odom_intriusic_publisher.publish(state.odom_intriusic)
