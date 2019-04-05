#!/usr/bin/env python

from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import  *
from sensor_msgs.msg import *
from nav_msgs.msg import  *
from math import *
from sensor_msgs.msg import Joy
import rospy



# the joystick is a script that convert the axis from the joystick to linear and angular
# velocity and publish it the joystick script also change the param "robot_status"
#to the task hardcoded to each button





#class for callback
class Callbacks():
    def __init__(self):
        self.status = 'none'
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.state = "teleop"
        self.max_linear_velocity = 2
        self.max_angular_velocity = 2

    def Joystick_callback(self,data):
        self.twist.linear.x = data.axes[1] * self.max_linear_velocity
        self.twist.angular.z = data.axes[0] *self.max_angular_velocity
        self.cmd_publisher.publish(self.twist)
        if data.buttons[2] == 1:
            print("joystick teleop")
            self.state = "teleop"
        elif data.buttons[0] == 1:
            print("joystick fms")
            self.state = "fms"

# main with ros
if __name__ == '__main__':
    rospy.init_node('JoyStick')
    my_callbacks = Callbacks()
    rate = rospy.Rate(50)

    rospy.Subscriber('joy', Joy, my_callbacks.Joystick_callback)
    robot_status_pub = rospy.Publisher('/robot_status', String, queue_size=20 )

    while not rospy.is_shutdown():
        robot_status_pub.publish(my_callbacks.state)
