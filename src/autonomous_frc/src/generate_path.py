#!/usr/bin/env python

from cubic_spline import spline
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import  *
from sensor_msgs.msg import *
from nav_msgs.msg import  *
from math import *
import tf
import rospy


class callback_functions:
	# The values here are not mandatory, we can choose to initialize whatever we want.
	# I just gave a few things as an example. You better move on to the main function and come back here later:
    def __init__(self):
        self.header=Header()
        self.x=0
        self.y=0
        self.theta=0
        self.point = [0,0,0]

    def change(self,data):
        d = json.loads(data.data)
        print(d)
        self.point[0]=  d['x']
        self.point[1] = d['y']
        self.point[2] = d['theta']

    def nav_goal_callback(self,data):
        x_goal=data.pose.position.x
        y_goal=data.pose.position.y
        dist=sqrt((self.x-x_goal)**2+(self.y-y_goal)**2)
        quaternion = data.pose.orientation
        goal_heading = tf.transformations.euler_from_quaternion((quaternion.x,quaternion.y,quaternion.z,quaternion.w))
        theta_goal=degrees(goal_heading[2])
        theta_start=degrees(atan2(y_goal-self.y,x_goal-self.x))
        points = spline((self.x,self.y,theta_start),(x_goal,y_goal,theta_goal),dist*2)
        kp = []
        for p in points:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "/map"
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0
            kp.append(pose)            

        path.poses = kp
        path.header.frame_id = "/map"
        path.header.stamp = rospy.Time.now()
        # path.PoseStamped = kp
        path_publisher.publish(path)

    def odom_callback(self,data):
        self.x=data.pose.pose.position.x
        self.y=data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        my_heading = tf.transformations.euler_from_quaternion((quaternion.x,quaternion.y,quaternion.z,quaternion.w))
        self.theta=degrees(my_heading[2])




# (16.5,6.7,0) end point
if __name__ == '__main__':
    rospy.init_node('path_generator')
    rate = rospy.Rate(1)
    functions_object=callback_functions()
    path_publisher = rospy.Publisher('/path', Path, queue_size=20)
    nav_goal_subscriber=rospy.Subscriber('/move_base_simple/goal', PoseStamped, functions_object.nav_goal_callback)
    odom_subscriber=rospy.Subscriber('/odom', Odometry, functions_object.odom_callback)
    path = Path()
    rospy.set_param("/robot_status","follow")
    while not rospy.is_shutdown():
        rospy.spin()
        
