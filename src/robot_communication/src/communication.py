#!/usr/bin/env python

import subprocess
from threading import Thread
from subprocess import Popen, PIPE, check_output
from sys import argv, stdout
import json
from std_msgs.msg import Header,String
import psutil
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import  *
from sensor_msgs.msg import *
from nav_msgs.msg import  *
from math import *
import tf
from networktables import NetworkTables
global com,json_data

# receives odom data from roborio and from /cmd_vel,
# publishes odom and status and sends to roborio cmd_vel

def getBattery():
    try:
        return str(int(psutil.sensors_battery().percent))
    except:pass
    return "Unable"

def main():
    #vars


    #ros init
    rospy.init_node('communication_node')
    odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=100)
    status_publisher =  rospy.Publisher('/robot_status', String, queue_size=100)
    cmd_object = callback()
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_object.cmd_vel_callback)
    # cmd_vel_sub_2 = rospy.Subscriber('/cmd_vel2', Twist, cmd_object.callback_function2)
    rate = rospy.Rate(50)

    #communication init
    global com
    global json_data
    json_data =None
    com = communication()
    while not rospy.is_shutdown():
        com.sd.putString("battery",getBattery())
        Data_json = json_data

        # print(Data_json)
        if Data_json != None:
            current_time=rospy.Time.now()


            odom_broadcaster=tf.TransformBroadcaster()

            odom=Odometry()
            odom.header.frame_id='/odom'
            odom.child_frame_id='/base_link'

            theta = radians(Data_json['robotadrive']['odometry']['theta'])
            x = Data_json['robotadrive']['odometry']['x']
            y = Data_json['robotadrive']['odometry']['y']
            linear = Data_json['robotadrive']['odometry']['linear_velocity']
            angular = Data_json['robotadrive']['odometry']['angular_velocity']
            distance = Data_json['robotadrive']['odometry']['distance']
            odom_quat=tf.transformations.quaternion_from_euler(distance,0,-theta)
            odom_quat_tf=tf.transformations.quaternion_from_euler(0,0,-theta)
            odom_broadcaster.sendTransform((x,y,0), odom_quat_tf, current_time, '/base_link', '/odom')
            odom.header.stamp=current_time
            odom.pose.pose=Pose(Point(x,y,0),Quaternion(*odom_quat))
            status_publisher.publish(Data_json['robot_status'])
            odom_publisher.publish(odom)

        rate.sleep()





class callback:
    def __init__(self):
        self.linear, self.angular = 0, 0
    def cmd_vel_callback(self, data):
        self.linear = data.linear.x
        self.angular = data.angular.z
        global com
        com.sd.putString("command",json.dumps({'drive':{'v':self.linear,'w': self.angular}}))
    def ref_tape_callback(self, data):
        self.linear = data.linear.x
        self.angular = data.angular.z
        global com

        com.sd.putString("command",json.dumps({'hatch':{'distance':self.linear,'pixel': self.angular}}))



def valueChanged(table, key, value, isNew):
    # print(value)
    if key == "json":
        global json_data
        json_data = json.loads(value)
class communication:
    def __init__(self):
        NetworkTables.initialize(server='roborio-2230-frc.local')
        self.sd=  NetworkTables.getTable("database")
        self.sd.addEntryListener(valueChanged)

if __name__ == '__main__':
	main()
