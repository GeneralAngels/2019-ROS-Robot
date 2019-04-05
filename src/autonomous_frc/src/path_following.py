#!/usr/bin/env python
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import  *
from sensor_msgs.msg import *
from nav_msgs.msg import  *
from math import *
import tf
import rospy
from time import time


def sign(num):
    if num < 0:
        return -1
    else:
        return 1

class Path_following:
    def __init__(self):
        self.kv = 0.8
        self.kwf =5# omega forward
        self.kwb = 5 # omega backwards
        self.maxw = 3
        self.ki = 0
        self.integral = 0
        self.maxv=  2
        self.path = Path()
        self.path.poses.append(PoseStamped())
        self.point_index = 0
        self.min_radius = 0.3
        self.my_odometry = Odometry()
        self.last_error_heading = 0
        self.length = 0
        self.dis=1
        self.d_kwf =5.5
        self.d_kwb = 5.5
        self.d_kv =  2

    def calc_errors(self):
        quaternion = self.my_odometry.pose.pose.orientation
        my_heading = tf.transformations.euler_from_quaternion((quaternion.x,quaternion.y,quaternion.z,quaternion.w))[2]
        my_point = self.my_odometry.pose.pose.position
        # print(my_heading)
        target_point = self.path.poses[self.point_index].pose.position
        # print(target_point.x,target_point.y)
        error_position =  sqrt((my_point.x - target_point.x)**2+(my_point.y - target_point.y)**2)
        target_heading = atan2((target_point.y - my_point.y),(target_point.x - my_point.x))
        error_heading =  my_heading - target_heading
        # print("error" + str(error_heading))
        if error_heading>3.14159:
            error_heading-=6.2832
        if error_heading<-3.14159:
            error_heading+=6.2832
        # if error_heading >= 0.05:
        #     self.kwf,self.kwb=4.5,4.5
        # else:
        #     self.kwf,self.kwb=6,6
        print(self.point_index,self.length)
        if self.length >0:
            end_point = self.path.poses[-1].pose.position
            dis = sqrt((my_point.x -end_point.x)**2 + (my_point.y-end_point.y)**2)
            first_dis = sqrt((end_point.x)**2 + (end_point.y)**2)
            print((0.125-(dis/(first_dis*8))))
            self.kwf =self.d_kwf *((0.125-(dis/(first_dis*8)))+0.9)
            self.kwb =self.d_kwb *((0.125-(dis/(first_dis*8)))+0.9)
            # print(dis)
            if dis <= 1.2:
                self.kwf +=(6 * 15*(0.065- (error_heading/7.5)))
                self.kwb +=( 6*15* (0.065- (error_heading/7.5)))
                self.kv = 1

                print("tom")
            else:
                self.kv = self.d_kv
            print(self.kwf)

        # if self.point_index >= self.length-25:
        #     self.kwf,self.kwb=4,
        # else:
        #     self.kwf =2# omega forward
        #     self.kwb = 2 # omega backwards
        print(error_position,error_heading)
        self.integral += (error_heading+self.last_error_heading) * self.dt/2
        self.last_error_heading = error_heading
        return (error_position,error_heading)

    def calc_signal(self,errors):

        direction = sign(cos(errors[1]))
        # direction=1
        signal_linear_velocity = errors[0] * direction * self.kv
        if direction>0:
            signal_angular_velocity = errors[1] * self.kwf
        else:
            signal_angular_velocity = errors[1] * self.kwb
        # print(abs(signal_linear_velocity))
        # signal_angular_velocity+=self.integral * self.ki
        if abs(signal_linear_velocity) > self.maxv:
            signal_linear_velocity = self.maxv * sign(signal_linear_velocity)
            # print(signal_linear_velocity)
        if abs(signal_angular_velocity) > self.maxw:
            signal_angular_velocity = self.maxw * sign(signal_angular_velocity)
        if self.point_index < 4:
            signal_angular_velocity*=1.1
        # if abs(errors[1]) >= 0.35:
        #      signal_linear_velocity*=0.5
        #      signal_angular_velocity = signal_angular_velocity/2.7


        return (signal_linear_velocity,signal_angular_velocity)

    def calc_point_index(self):
        my_point = self.my_odometry.pose.pose.position
        target_point = self.path.poses[self.point_index].pose.position
        if sqrt((my_point.x - target_point.x)**2+(my_point.y - target_point.y)**2) <= self.min_radius:
            self.point_index +=1

    def path_callback(self,path):
        self.path = path
        self.point_index =  0
        self.length = len(self.path.poses)

    def odom_callback(self,odom):
        if rospy.get_param('autonumos'):
            self.my_odometry = odom

    def odom_intriusic_callback(self,odom):
        if rospy.get_param('image_processing'):
            self.my_odometry = odom
    # def dis_callback(self,dis):
    #     self.dis = dis



if __name__ == '__main__':
    rospy.set_param("teleop",False)
    rospy.set_param("autonumos",False)
    rospy.set_param("image_processing",False)
    rospy.init_node('path_following')
    my_follower = Path_following()
    rate = rospy.Rate(50)
    rospy.Subscriber('path', Path, my_follower.path_callback)
    rospy.Subscriber('odom', Odometry, my_follower.odom_callback)
    rospy.Subscriber('odom_intriusic',Odometry,my_follower.odom_intriusic_callback)
    cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=20 )
    path_status_publisher = rospy.Publisher('/path_status', String, queue_size=20 )
    my_marker_publisher = rospy.Publisher('/my_marker', PointStamped, queue_size=20 )
    point_marker_publisher = rospy.Publisher('/point_marker', PointStamped, queue_size=20 )
    arrow_publisher = rospy.Publisher('/arrow', Marker, queue_size=10)
    drive_command = Twist()
    stop_flag=False
    my_follower.dt = 0.02
    while not rospy.is_shutdown():
        # print(rospy.get_param("teleop"))
        if not rospy.get_param("teleop"):
            if len(my_follower.path.poses) <= my_follower.point_index:
                if stop_flag:
                    drive_command.angular.z = 0
                    drive_command.linear.x = 0
                    cmd_publisher.publish(drive_command)
                    stop_flag=False
                    print("done")
                    path_status_publisher.publish("done")
                    my_follower.integral =0

            else:
                start_time = time()
                point_data=PointStamped()
                point_data.header.stamp=rospy.Time.now()
                point_data.header.frame_id='map'
                point = my_follower.path.poses[my_follower.point_index].pose.position
                point_data.point = Point(point.x,point.y,0.5)
                point_marker_publisher.publish(point_data)
                point = my_follower.my_odometry.pose.pose.position
                point_data.point = Point(point.x,point.y,0.5)
                my_marker_publisher.publish(point_data)
                quaternion = my_follower.my_odometry.pose.pose.orientation
                mark = Marker()
                mark.header.frame_id='/map'
                mark.action = mark.ADD
                mark.type = mark.ARROW
                mark.id = 0
                mark.pose.orientation= quaternion
                mark.pose.position = point
                mark.pose.position.z = 0.6
                mark.scale.x = 0.5
                mark.scale.y =0.05
                mark.scale.z = 0.05
                mark.color.a=1.0
                arrow_publisher.publish(mark)
                errors = my_follower.calc_errors()
                linear , angular =  my_follower.calc_signal(errors)
                drive_command.angular.z =-angular
                drive_command.linear.x = linear
                cmd_publisher.publish(drive_command)
                my_follower.calc_point_index()
                stop_flag=True
                my_follower.dt = time()-start_time
        rate.sleep()
