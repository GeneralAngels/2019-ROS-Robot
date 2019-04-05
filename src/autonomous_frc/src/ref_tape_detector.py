#!/usr/bin/env python

#imports
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import  *
from sensor_msgs.msg import *
from nav_msgs.msg import  *
import numpy as np
import cv2 as cv
from math import *
from cv_bridge import CvBridge, CvBridgeError
import random
global create
import tf
import bisect

#global vars setup:
publish_img = True
publish_path = True
create = True

#callback class made for ros image object - depth, raw depth and infra-red mat.
class my_image:
    def __init__(self):
        self.bridge1 = CvBridge()
        self.bridge2 = CvBridge()
        self.ir_sub = rospy.Subscriber('/camera/ir/image', Image, self.ir_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth_registered/image_raw', Image, self.depth_callback)
        self.ir = None
        self.depth = None

    def ir_callback(self, data):
        try:
            data.encoding = "mono16"
            self.ir = self.bridge1.imgmsg_to_cv2(data, "mono16")
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        try:
            # print(data)
            depth = self.bridge2.imgmsg_to_cv2(data, "passthrough")
            self.depth = depth.astype(np.float32)

        except CvBridgeError as e:
            print(e)


# main function
def analyze():
    #bridge - image compression for ros: required for publishing and subscribing to Image topics
    bridge = CvBridge()

    # colors = [get_rand_color() for i in range(8)]
    colors = [(0,0,255),(0,255,0),(255,0,0), (0,255,255),(255,0,255),(255,255,0)]

    #inits node
    rospy.init_node('ir_analyzer_test')
    np.set_printoptions(suppress=True)
    rate = rospy.Rate(20)

    #ros subscribers and publishers setup
    image = my_image()
    dis_pub  = rospy.Publisher('/dis', Float32, queue_size=30)
    if publish_img:
        image_pub  = rospy.Publisher('/camera/ir/proc', Image, queue_size=30)
    if publish_path:
        path_pub  = rospy.Publisher('/path', Path, queue_size=30)
    font = cv.FONT_HERSHEY_SIMPLEX
    min_mean_depth = 20000
    last_publish = False

    while not rospy.is_shutdown():
        depth = image.depth
        ir = image.ir
        # print(depth)
        # and cv.mean(depth)[0] > min_mean_depth
        if depth is not None and ir is not None :
            #image size:
            rangeD_ = cv.inRange(depth, 0, 50)


            #takes only high hue points
            range_ = cv.inRange(ir, 1020, 1022)
            colored = cv.cvtColor(range_, cv.COLOR_GRAY2RGB)
            # depth = cv.cvtColor(rangeD_, cv.COLOR_GRAY2RGB)

            #smoothing and bluring noise
            kernal = np.ones((1,1))
            colored = cv.erode(colored, kernal, iterations = 3)
            colored = cv.dilate(colored, np.ones((1,1)), iterations = 2)
            kernel = np.ones((5,5),np.float32)/49
            colored = cv.filter2D(colored,-1,kernel)
            # colored = cv.bilateralFilter(colored,9,75,75)
            colored = cv.erode(colored, np.ones((3,3)), iterations = 2)
            # print(colored)
            colored = cv.inRange(colored, (20,20,20),(255,255,255))
            colored = cv.cvtColor(colored, cv.COLOR_GRAY2RGB)

            #contours
            ret,thresh = cv.threshold(colored,20,1022,0)
            thresh = cv.cvtColor(thresh, cv.COLOR_RGB2GRAY)
            _, contours,hierarchy = cv.findContours(thresh, 1, 2)

            squares = []
            a = 0
            for cnt in contours:
                square, angle, check = get_square(cnt, 0.75)
                if check and not int(angle) == 0:
                    x, y = get_center(square)
                    squares.append((square, angle, (x,y)))
                    a += 1
            # squares = squares.tolist()
            x = lambda x_p:x_p[2][0]
            # squares.sort(key=x)
            squares = sorted(squares, key=x)
            err = 3.5
            NofCo = int(len(squares)/2)
            cop = []
            for i in range(len(squares) - 1):
                s1, a1, c1 = squares[i]
                s2, a2, c2 = squares[i+1]
                # cv.putText(colored,str(int(i)),c1, font, 1,(255,255,255),2,cv.LINE_AA)
                # check1 = abs(a1) - err < abs(a2) < abs(a1) + err
                check2 = is_couple(s1.copy(), s2.copy())
                cv.putText(colored,str(int(a1)),(s1[0][0], s1[0][1]), font, 1,(255,255,255),2,cv.LINE_AA)
                cv.putText(colored,str(int(a2)),(s2[0][0], s2[0][1]), font, 1,(255,255,255),2,cv.LINE_AA)
                if check2:
                    cop.append((s1, s2))
                    i+=1


            sort_by_dis = lambda a : get_dis(get_center(a[0]), get_center(a[1]))
            cop = sorted(cop, key=sort_by_dis, reverse=True)

            if len(cop) <= NofCo and len(cop) > 0:
                closest = cop[0]
                rest = cop[1:]
                for cnt in rest:
                    cv.drawContours(colored, [cnt[0]], 0, colors[2], 3)
                    cv.drawContours(colored, [cnt[1]], 0, colors[2], 3)

                cv.drawContours(colored, [closest[0]], 0, colors[1], 3)
                cv.drawContours(colored, [closest[1]], 0, colors[1], 3)


                publish = rospy.get_param('image_processing')

                # publish = True
                # last_publish = False
                path,dis = get_path(closest, depth)
                # dis_pub.publish(dis)
                # print(publish)
                if publish and not last_publish:

                    # print(path)
                    if path is not None:
                        path_pub.publish(path)
                last_publish = publish


            # for cnt in contours:
            #     cv.drawContours(colored, [cnt], 0, (0,0,255), 2)
            # print(depth)
            try:
                if publish_img:
                    image_pub.publish(bridge.cv2_to_imgmsg(colored,'rgb8'))
            except Exception as e:
                print('wrong format: ', str(e))
        rate.sleep()

#---f--kin-----l---it----------------------------------------------------------------------------------#
#----uc---g--bu-lsh------------------------------------------------------------------------------------#
#---------------------------------------------useful functions----------------------------------------#
#-----------------------------------------------------------------------------------------------------#
global last_a, last_x, last_y, last_t
last_a = 0
last_x = 0
last_y = 0
last_t = 0
def get_path(closest, depth):
    global last_a, last_x, last_y, last_t
    xl, yl = get_center(closest[0])
    xr, yr = get_center(closest[1])

    xl -= 20
    xr += 20
    xl = limit(xl)
    xr = limit(xr)
    # print(xl)
    dis_left, dis_right = depth[yl][xl], depth[yr][xr]

    dis_to_tape = (dis_left + dis_right) / 2
    print(dis_left, dis_right)
    O1 = calc_dist_from_target((xl,yl), 0, 640)[1]
    O2 = calc_dist_from_target((xr,yr), 0, 640)[1]

    rxl, ryl = dis_left * cos(O1), dis_left * sin(O1)
    rxr, ryr = dis_right * cos(O2), dis_right * sin(O2)
    # print(ryl ,ryr, rxl, rxr)
    tape_angle = - degrees(atan2(ryl - ryr, rxl - rxr) + 1.56)


    alpha = 0.4
    t_alpha = 0.05
    width = 640
    centerP = [get_center(cnt) for cnt in closest]
    dis = get_dis(centerP[0], centerP[1])
    cntr = get_center_point(centerP[0], centerP[1])
    _, angle_to_tape = calc_dist_from_target(cntr, dis, width)
    tape_angle += degrees(angle_to_tape)
    # print tape_angle
    angle_to_tape = angle_to_tape*alpha + last_a*(1-angle_to_tape)

    # print('alpha:', angle_to_tape,'\n beta:', tape_angle, '\n distance', dis_to_tape)
    if publish_path:
        x = (dis_to_tape * cos(angle_to_tape))
        x = (x*alpha) + (last_x*(1-alpha))
        y = - (dis_to_tape * sin(angle_to_tape))
        y = (y*alpha) + (last_y*(1-alpha))
        # print(x,y)
        # rad_tape = radians(tape_angle)
        #
        rad_tape = (atan2(y,x)*0.9+radians(tape_angle)*0.1)
        xe = x - ((dis_to_tape - 0.4) * sqrt(1-(sin(rad_tape)**2)))

        ye = y - ((dis_to_tape-0.4)* sin(rad_tape))
        xd = x + (1.1* sqrt(1-(sin(rad_tape)**2)))

        yd = y + (1.1 * sin(rad_tape))
        # distance_to_e = sqrt((xe**2) + (ye**2))
        k= dis_to_tape*1.2

        # print(k)
        # print((2*x-k*cos(rad_tape))/k)
        # print(acos((2*xe-k*cos(rad_tape))/k))
        # print(asin((2*ye-k*sin(rad_tape))/k))

        # print(y)
        # if y<0:
        #     first_angle =rad_tape + angle_to_tape
        # else:
        #     first_angle =-rad_tape -angle_to_tape
        # print(-asin(xe/distance_to_e))
        # print("tape",tape_angle,(1 * sqrt(1-(sin(rad_tape)**2))),(1 * sin(rad_tape)))
        # print('x: ' + str(x) + '\ny: ' + str(y) + '\nfirst alpha: ' + str(angle_to_tape) + '\nlast alpha: ' + str(tape_angle))
        # cubic_spline((xe,ye,tape_angle),(xd,yd,tape_angle),1,75.0)

        cubic_path = getSplinePath((0,0), (xe,ye),(xd, yd) )
        # print(len(cubc_path))
        # cubic_path = cubic_spline((0,0,degrees(start_angle)),(xe,ye,tape_angle),dis_to_tape,75.0) +cubic_spline((xe,ye,tape_angle),(x,y,tape_angle),1.2,75.0)
        # cubic_path = cubic_spline((0,0,180),(-0.1,-0.1,asin(xe/distance_to_e)),sqrt(0.1**2+0.1**2),100.0) + cubic_spline((-0.1,-0.1,asin(xe/distance_to_e)),(xe,ye,tape_angle),dis_to_tape,75.0) + cubic_spline((xe,ye,tape_angle),(xd,yd,tape_angle),1,20.0)
        # print(cubic_path)
        # prisnt(cubic_path)
        path = cubic_2_path(cubic_path)

        create = True
        if not str(x)=='nan' and not str(y)=='nan' :
            last_x = x
            last_y = y
            return path,dis_to_tape
        else:
            return None,0

#returns the distance and the angle from a reflective tape target
def calc_dist_from_target(center_point, target_width, image_width):
    image_width = 640 #Pixels
    camera_rat = 60 #degrees
    target_original_size = 30 #cm

    angle_mult = 0.5
    try:
        deg_from_camera = ((float(center_point[0])/float(image_width))*float(camera_rat) - (camera_rat/2)) * angle_mult #degrees
        deg_from_camera = deg_from_camera * angle_mult
    except:
        deg_from_camera = 0.0001

    dis = ((5*pow(10,-5))*pow(target_width,2)) - (0.0248 * target_width) + 3.6823
    return dis, deg_from_camera * 0.2

def limit(a):
    if a >= 640:
        return 639
    elif a <= 0:
        return 0
    else: return a

#returns random 3 bit vector (rgb)
def get_rand_color():
    return (random.randint(0,255),random.randint(0,255),random.randint(0,255))

#checks if the two squares have a avalid distance between them
def squareCheck(s1, s2, d):
    l1, l2, r1, r2 = s1[0][1], s1[0][1] ,s2[0][1] ,s2[0][1]
    for p in s1:
        if p[1] < l1:
            l1 = p[1]
        if p[1] > r1:
            r1 = p[1]
    for p in s2:
        if p[1] < l2:
            l2 = p[1]
        if p[1] > r2:
            r2 = p[1]
    d1, d2 = r1 - l1, r2 - r1
    ev = (d1 + d2)/2
    return ev < d


#returns the distance between two points
def get_dis(p1,p2):
    return sqrt((p2[1] - p1[1])**2 + (p2[0] - p1[0])**2)

# def is_couple(c1, c2):
#     print(c1)
#     c1 = c1.tolist()
#     c2 = c2.tolist()
#     c1.sort(key = lambda x: x[1])
#     c2.sort(key = lambda x: x[1])
#     print(c1)
#     xh1 = get_center_point(c1[0], c1[1])[0]
#     xh2 = get_center_point(c2[0], c2[1])[0]
#     xl1 = get_center_point(c1[2], c1[3])[0]
#     xl2 = get_center_point(c2[2], c2[3])[0]
#     dh = abs(xh1 - xh2)
#     dl = abs(xl1 - xl2)
#     print(dl, dh)
#     return dl < dh

def is_couple(cnt1, cnt2):
    s1, a1 = get_mass_dir(cnt1)
    s2, a2 = get_mass_dir(cnt2)
    err = 3.0
    norm_check = lambda a : 0 <= abs(a) <= 9
    check = not a1 > 0 > a2
    return check and norm_check(a1) and norm_check(a2)

#returns the angle between two points
def get_angle(p1, p2):
    a = float(p1[0])
    b = float(p2[0])
    c = float(p1[1])
    d = float(p2[1])
    try:
        m = (d-c)/(b-a)
        return m
    except:
        return 0

#returns the idial square for the given contours and the angle (direction) of the mass
def get_mass_dir(cnt):
    rect = cv.minAreaRect(cnt)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    max = 0
    a = 0
    for i in range(3):
        dis = get_dis(box[i], box[i+1])
        if dis > max:
            max = dis
            a = get_angle(box[i], box[i+1])
    return box, a


#checks if the given contours are in range of 30 precent of the perfect found square
def get_square(cnt, err_rate):
    min_area = 160
    area = cv.contourArea(cnt)
    area_size_check = area > min_area
    cnt_len = cv.arcLength(cnt, True)
    approx = cv.approxPolyDP(cnt, 0.02*cnt_len, True)
    square, angle = get_mass_dir(cnt)
    square_area = cv.contourArea(square)
    try:
        area_diff = square_area/area
    except:
        area_diff = 0
    refer = (1 - err_rate < area_diff < 1 + err_rate) and area_size_check
    return square, angle, refer

#returns the center points of contours
def get_center(cnt):
    if len(cnt) > 0:
        try:
            sumX, sumY, c = 0, 0, 0
            for point in cnt:
                sumX += point[0][0]
                sumY += point[0][1]
                c += 1
            cX, cY = sumX/c, sumY/c
            return (cX, cY)
        except:
            sumX, sumY, c = 0, 0, 0
            for point in cnt:
                sumX += point[0]
                sumY += point[1]
                c += 1
            cX, cY = sumX/c, sumY/c
            return (cX, cY)
    else:
        return 0, 0

#returns the center point between two point
def get_center_point(p1, p2):
    cx = (p1[0] + p2[0]) / 2
    cy = (p1[1] + p2[1]) / 2
    return (cx,cy)

# makes sure that the distance between the couples is related to the area of the couples
def areas_check(area1, area2, dist):
    err = 2.8
    return 1/err <= (area1/area2) <= 1*err

#claculates the angle of the reflective tape
def calc_tapes_deg(cnt_left, cnt_right, angle_to_target):
    magic_num = 0.1
    top_left = top_right = (1024,0)
    btn_left = btn_right = (0,0)

    for i in range(3):
        if cnt_left[i][0] < top_left[0]:
            top_left = cnt_left[i]
        if cnt_left[i][0] > btn_left[0]:
            btn_left = cnt_left[i]
        if cnt_right[i][0] < top_right[0]:
            top_right = cnt_right[i]
        if cnt_right[i][0] > btn_right[0]:
            btn_right = cnt_right[i]

    right_dist = top_right[0] - btn_right[0]
    left_dist = top_left[0] - btn_left[0]
    diff = (float(right_dist) - float(left_dist)) * 7 - float(angle_to_target) * magic_num
    return diff



#cubic spline path creator
def cubic_spline(spoint,epoint,k,n,pow2=False):
    dx = spoint[0]
    dy = spoint[1]
    cx  = k * cos(radians(spoint[2]))
    cy =k * sin(radians(spoint[2]))
    bx = 3*epoint[0]- 3*spoint[0] - k * cos(radians(epoint[2])) - 2*k*cos(radians(spoint[2]))
    by = 3*epoint[1]- 3*spoint[1] - k* sin(radians(epoint[2])) - 2*k*sin(radians(spoint[2]))
    if pow2:
        ax=0
        ay=0
    else:
        ax = k * cos(radians(epoint[2])) -  2*epoint[0]+ k * cos(radians(spoint[2])) + 2*spoint[0]
        ay = k * sin(radians(epoint[2])) -  2*epoint[1]+ k * sin(radians(spoint[2])) + 2*spoint[1]
    points = []
    for s in range(0,int(n+1),1):
        point = [ax*pow((s/n),3) + bx*pow((s/n),2) + cx*(s/n) + dx,ay*pow((s/n),3) + by*pow((s/n),2) + cy*(s/n) + dy]
        points.append(point)
    return points

N = 3
def bspline_planning(x, y, sn):
    t = range(len(x))
    x_tup = si.splrep(t, x, k=N)
    y_tup = si.splrep(t, y, k=N)
    x_list = list(x_tup)
    xl = x
    x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]
    y_list = list(y_tup)
    yl = y
    y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]
    ipl_t = np.linspace(0.0, len(x) - 1, sn)
    rx = si.splev(ipl_t, x_list)
    ry = si.splev(ipl_t, y_list)
    return rx, ry

def main():
    x = np.array([-1.0, 3.0, 4.0, 2.0, 1.0])
    y = np.array([0.0, -3.0, 1.0, 1.0, 3.0])
    sn = 100
    rx, ry = bspline_planning(x, y, sn)
    plt.plot(x, y, '-og', label="Waypoints")
    plt.plot(rx, ry, '-r', label="B-Spline path")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.show()

class Spline:
    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []
        self.x = x
        self.y = y
        self.nx = len(x)
        h = np.diff(x)
        self.a = [iy for iy in y]
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None
        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0
        return result

    def calcd(self, t):
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None
        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None
        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]
        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h):
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B

class Spline2D:
    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [sqrt(idx ** 2 + idy ** 2)
                   for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        x = self.sx.calc(s)
        y = self.sy.calc(s)
        return x, y

    def calc_curvature(self, s):
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_yaw(self, s):
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = atan2(dy, dx)
        return yaw

def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))
    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))
    return rx, ry, ryaw, rk, s


def getSplinePath(p1, p2, p3):
    x = [p1[0], p2[0], p3[0]]
    y = [p1[1], p2[1], p3[1]]

    ds = 0.1
    sp = Spline2D(x, y)
    s = np.arange(0, sp.s[-1], ds)
    rx, ry,  = [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
    path = [(rx[i], ry[i]) for i in range(len(rx))]
    return path

def circle(xe,ye,rad_tape):
    c = (-xe/tan(rad_tape))+ye
    m= -1/tan(rad_tape)
    t = (xe**2 -2*ye*c + ye**2)/(2*xe+2*ye * m)
    start_angle = atan(-t/(t*m+c))
    r = sqrt(t**2+(t*m+c)**2)
    points =[]
    for i in range(0,101):
        points+= [[sqrt((r**2)-((((i/100.0)*ye)-t)**2))+t*m+c,(i/100.0)*ye]]
        # print([[(i/100.0)*xe,sqrt((r**2)-((((i/100.0)*xe)-t)**2))+t*m+c]])
    return points
#converts points array to ros Path
def cubic_2_path(points,offsets=[0,0]):
    path = Path()
    poses = []
    for x,y in points:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "/map"
        pose.pose.position.x = x - offsets[0]
        pose.pose.position.y = y - offsets[1]
        pose.pose.position.z = 0.13
        poses.append(pose)
    path.poses = poses
    path.header.frame_id = "/map"
    path.header.stamp = rospy.Time.now()
    return path


#calling main function
if __name__ == '__main__':
    analyze()
