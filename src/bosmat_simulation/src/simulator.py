#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import  *
from sensor_msgs.msg import *
from nav_msgs.msg import  *
import tf
from math import *
import time
import numpy as np

class Motor():
    def __init__(self,alpha):
        self.Alpha = alpha
        self.PrevVal = 0
        self.CurrentVal = 0

    def filter(self,currentVal):
        self.CurrentVal = currentVal * (1- self.Alpha) + self.PrevVal * self.Alpha
        self.PrevVal = self.CurrentVal
        return self.CurrentVal

class Robot():
    def __init__(self,alpha,width,wheelRadius):
        self.MotorRight = Motor(alpha)
        self.MotorLeft = Motor(alpha)
        self.Width = width
        self.WheelRadius = wheelRadius

    def drive(self,Vr,Vl):
        TrueOmegaR,TrueOmegaL = self.MotorRight.filter(Vr),self.MotorLeft.filter(Vl)
        return TrueOmegaR,TrueOmegaL

class callback_functions:
    def __init__(self):
        self.magicNumber = 0.01
        self.angular=0
        self.linear=0
        self.MAKBILIT_angular = 0
        self.theta=0
        self.x=0
        self.y=0
        self.override_tf=False
        self.quat=[0,0,0,1]
        self.max_linear_velocity = 3
        self.max_angular_velocity = 3

    def pose_callback(self, data):
        self.x=data.pose.pose.position.x
        self.y=data.pose.pose.position.y
        self.theta=data.pose.pose
        orientation=data.pose.pose.orientation
        self.quat=[orientation.x,orientation.y,orientation.z,orientation.w]
        self.override_tf=True

    def drive_callback(self, data):
        self.angular=constrain_value(data.angular.z,-self.max_angular_velocity,self.max_angular_velocity)
        self.linear=constrain_value(data.linear.x,-self.max_linear_velocity,self.max_linear_velocity)        

    def MAKBILIT_callback(self, data):
        self.MAKBILIT_angular = data.angular.y* self.magicNumber

def constrain_value(value,min_val,max_val):
    if value>max_val:
        value=max_val
    if value<min_val:
        value=min_val
    return value


if __name__ == '__main__':

    rospy.init_node('frc_simulator')
    np.set_printoptions(suppress=True)
    robot = Robot(0.9,0.54,0.106)
    function_object = callback_functions()
    vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, function_object.drive_callback)
    pose_subscriber = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, function_object.pose_callback)
    # vel_subscriber = rospy.Subscriber('/MAKBILIT', Twist, function_object.MAKBILIT_callback)
    # joint_publisher = rospy.Publisher('joint_states', JointState,queue_size=10)

    robotJoints = JointState()
    robotJoints.name = ['MAKBILIT_to_body','gripper_to_MAKBILIT','center_object_to_robot','center_object_to_car']

    odom_broadcaster=tf.TransformBroadcaster()
    odom_publisher=rospy.Publisher('odom',Odometry,queue_size=10)
    odom=Odometry()
    odom.header.frame_id='odom'
    odom.child_frame_id='base_link'

    car_theta = 0
    x = 0
    y = 0
    theta = 0
    odom_quat=[0,0,0,1]
    MAKBILIT_angle = 0

    dt = 0.01
    rate = rospy.Rate(1/dt)
    

    while not rospy.is_shutdown():
        current_time=rospy.Time.now()
        # robot position and angle
        OmegaR_desired= (function_object.linear/robot.WheelRadius)-(function_object.angular * robot.Width)/(2*robot.WheelRadius)
        OmegaL_desired= (function_object.linear/robot.WheelRadius)+(function_object.angular * robot.Width)/(2*robot.WheelRadius)
        TrueOmegaR,TrueOmegaL= robot.drive(OmegaR_desired,OmegaL_desired)
        TrueVelocityR,TrueVelocityL= TrueOmegaR* robot.WheelRadius,TrueOmegaL* robot.WheelRadius
        TrueAngular= (TrueVelocityL-TrueVelocityR)/robot.Width
        TrueLinear= (TrueVelocityL+TrueVelocityR)/2
        if function_object.override_tf:
            odom_quat=function_object.quat
            theta=tf.transformations.euler_from_quaternion(odom_quat)[2]
            x=function_object.x
            y=function_object.y
            function_object.override_tf=False
        else:
            pass
            theta += TrueAngular * dt
            x +=TrueLinear * cos(theta) * dt
            y +=TrueLinear * sin(theta) * dt
            odom_quat=tf.transformations.quaternion_from_euler(0,0,theta)                

        odom_broadcaster.sendTransform((x,y,0), odom_quat, current_time, 'base_link', 'odom') #broadcast tf

        odom.header.stamp=current_time
        odom.pose.pose=Pose(Point(x,y,0),Quaternion(odom_quat[0],odom_quat[1],odom_quat[2],odom_quat[3]))
        odom.twist.twist=Twist(Vector3(TrueLinear,0,0),Vector3(0,0,TrueAngular)) #intrinsic
        odom_publisher.publish(odom) #publish odometry

        # robot joints only moving parts in the robot himself
        # MAKBILIT_angle += function_object.MAKBILIT_angular
        # if MAKBILIT_angle >=0.55:
        #     MAKBILIT_angle = 0.55
        # if MAKBILIT_angle <=-0.55:
        #     MAKBILIT_angle = -0.55
        # robotJoints.position = [MAKBILIT_angle,-MAKBILIT_angle,car_theta%3.14*2 - theta,0]
        # robotJoints.header.stamp=rospy.Time.now()
        # joint_publisher.publish(robotJoints)

        # car_theta+=0.004
        rate.sleep() # Wait fot 0.01 secs
