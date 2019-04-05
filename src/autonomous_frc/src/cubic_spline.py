import numpy as np
import math
def spline(a,b,k):
    # k = math.sqrt((a[0]-b[0])**2 + (a[1]+b[1])**2) * k
    dx = a[0]
    dy = a[1]
    cx  = k * math.cos(math.radians(a[2]))
    cy =k * math.sin(math.radians(a[2]))
    bx = 3*b[0]- 3*a[0] - k * math.cos(math.radians(b[2])) - 2*k*math.cos(math.radians(a[2]))
    by = 3*b[1]- 3*a[1] - k* math.sin(math.radians(b[2])) - 2*k*math.sin(math.radians(a[2]))
    ax = k * math.cos(math.radians(b[2])) -  2*b[0]+ k * math.cos(math.radians(a[2])) + 2*a[0]
    ay = k * math.sin(math.radians(b[2])) -  2*b[1]+ k * math.sin(math.radians(a[2])) + 2*a[1]
    # print(ax,ay,bx,by,cx,cy,dx,dy)
    # print((ax*math.pow((49/50),3) + bx*math.pow((49/50),2) + cx*(49/50) + dx,ay*math.pow((49/50),3) + by*math.pow((49/50),2) + cy*(49/50) + dy))
    points = []
    # x  = 2    
    for s in range(0,51,1):
        p = s/50.0
        point = (ax*math.pow(p,3) + bx*math.pow(p,2) + cx*p + dx,ay*math.pow(p,3) + by*math.pow(p,2) + cy*p + dy)
        # print(p,s)
        points.append(point)
    # print(points)
    return points

# import matplotlib.pyplot as plt
# points = spline((0,0,0),(100,0,90),0.125)
# x = []
# y = []
# for point in points:
#     x.append(point[0])
#     y.append(point[1])

# plt.plot(x, y, '-')
# plt.show()
