#!/usr/bin/env python

"""
Dubins curves written in python
Taken and readjusted from Andrew Walker's Dubins Curves in C
Written by Thomas Pound for the project Autopilot for Aerial Photography
Created 3/2/21
Last updated 3/2/21
"""

import math
import sys

class DubinsPath:
    """
    Dubins Path object, previously represented as a C Struct in dubins.c
    Parameters
        None
    Variables
        qi:         Initial coords
        params:     Length of each section
        rho:        Minimum turning radius
        path_type:  The type of path found in the "dubins_path_type" dictionary

    Function
        length:     Returns the entire length of the path including all segments
    """
    def __init__(self):
        self.qi = (-1,-1,-1)
        self.params = (-1,-1,-1)
        self.rho = -1
        self.path_type = -1

    def length(self):
        length = 0
        length += self.params[0]
        length += self.params[1]
        length += self.params[2]
        length = length*self.rho
        return length

class DubinsIntermediateResults:
    """
    Holds data from intermediate results
    """
    def __init__(self):
        self.alpha = 0
        self.beta = 0
        self.d = 0          # Distance
        self.sa = 0         # Sin alpha
        self.sb = 0         # Sin beta
        self.ca = 0         # Cos alpha
        self.cb = 0         # Cos beta
        self.c_ab = 0       # Cos alpha-beta
        self.d_sq = 0       # Distance squared

dubins_path_type = {"LSL":0,
                    "LSR":1,
                    "RSL":2,
                    "RSR":3,
                    "RLR":4,
                    "LRL":5}

seg_type = {"L_SEG":0,"S_SEG":1,"R_SEG":2}
path_segments = [[seg_type["L_SEG"],seg_type["S_SEG"],seg_type["L_SEG"]],
                [seg_type["L_SEG"],seg_type["S_SEG"],seg_type["R_SEG"]],
                [seg_type["R_SEG"],seg_type["S_SEG"],seg_type["L_SEG"]],
                [seg_type["R_SEG"],seg_type["S_SEG"],seg_type["R_SEG"]],
                [seg_type["R_SEG"],seg_type["L_SEG"],seg_type["R_SEG"]],
                [seg_type["L_SEG"],seg_type["R_SEG"],seg_type["L_SEG"]]]

def fmodr(x,y):
    return x - y*math.floor(x/y)

def mod2pi(theta):
    return fmodr(theta,2*math.pi)


points = []
def print_path(q,x):
    #print(f"x: {round(q[0],6)} y: {round(q[1],6)} theta: {round(q[2],6)} steps: {x}")
    #file.write(f"{round(q[0],6)},{round(q[1],6)},{round(q[2],6)}\n")
    points.append((round(q[0],6),round(q[1],6),round(q[2],6)))
    return 0

def dubins_segment(t,qi,segment):
    """
    Paramaters
        t:          StepSize
        qi:         Initial coords
        segment:    Current segment type of dubins path 
    """
    st = math.sin(qi[2])                # Sin theta
    ct = math.cos(qi[2])                # Cos theta

    if segment == seg_type["L_SEG"]:
        qt= (math.sin(qi[2]+t) - st,
            -math.cos(qi[2]+t) + ct,
            t)
    elif segment == seg_type["R_SEG"]:
        qt = (-math.sin(qi[2]-t) + st,
            math.cos(qi[2]-t) - ct,
            -t)
    elif segment == seg_type["S_SEG"]:
        qt = (ct*t,
            st*t,
            0.0)
    # Add translation back to the point
    qt = (qt[0] + qi[0],
        qt[1] + qi[1],
        qt[2] + qi[2])
    return qt

def dubins_path_sample(path,stepSize,q):
    tprime = stepSize/path.rho
    if stepSize<0 or stepSize> path.length():
        return 2
    qi = (0.0,0.0,path.qi[2])
    segments = path_segments[path.path_type]
    p1 = path.params[0]
    p2 = path.params[1]
    q1 = dubins_segment(p1,qi,segments[0])
    q2 = dubins_segment(p2,q1,segments[1])

    if tprime < p1:
        q = dubins_segment(tprime,qi,segments[0])
    elif tprime < (p1+p2):
        q = dubins_segment(tprime-p1,q1,segments[1])
    else:
        q = dubins_segment(tprime-p1-p2,q2,segments[2])

    q = (q[0]*path.rho+path.qi[0],
        q[1]*path.rho + path.qi[1],
        mod2pi(q[2]))

    return q


def dubins_path_sample_many(path, stepSize):
    global points
    points = []
    x = 0
    length = path.length()
    q = (-1,-1,-1)
    while x < length:
        q = dubins_path_sample(path,x,q)
        retcode = print_path(q,x)
        if retcode != 0:
            return retcode
        x += stepSize
    return points

def dubins_shortest_path(q0,q1,rho):
    best_cost = math.inf
    best_word = -1
    dub_ir = DubinsIntermediateResults()
    errcode = dubins_intermediate_results(dub_ir,q0,q1,rho)
    if errcode != 0:
        return -1
    
    path = DubinsPath()
    path.qi = q0
    path.rho = rho

    params = (-1,-1,-1) # Init params

    for i in range(0,6):
        params = dubins_word(dub_ir,i,params)
        if params != -1:
            cost = params[0] + params[1] + params[2]
            if cost < best_cost:
                best_word = i
                best_cost = cost
                path.params = params
                path.path_type = i
                print(params)
    if best_word == -1:
        return -1
    return path
                

def dubins_intermediate_results(dub_ir,q0,q1,rho):
    if rho <=0:
        return -1
    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    D = math.sqrt(dx*dx + dy*dy)
    d = D/rho
    theta = 0

    if d>0:
        theta = mod2pi(math.atan2(dy,dx))
    alpha = mod2pi(q0[2] - theta)
    beta = mod2pi(q1[2] - theta)

    dub_ir.alpha = alpha
    dub_ir.beta = beta
    dub_ir.d = d
    dub_ir.sa = math.sin(alpha)
    dub_ir.sb = math.sin(beta)
    dub_ir.ca = math.cos(alpha)
    dub_ir.cb = math.cos(beta)
    dub_ir.c_ab = math.cos(alpha-beta)
    dub_ir.d_sq = d*d

    return 0


def dubins_LSL(dub_ir, out):
    temp = dub_ir.d + dub_ir.sa - dub_ir.sb
    p_sq = 2+dub_ir.d_sq - (2*dub_ir.c_ab) + (2*dub_ir.d * (dub_ir.sa - dub_ir.sb))

    if p_sq>=0:
        temp1 = math.atan2((dub_ir.cb - dub_ir.ca),temp)
        out = (mod2pi(temp1-dub_ir.alpha),
                math.sqrt(p_sq),
                mod2pi(dub_ir.beta - temp1))
        return out
    return -1

def dubins_RSR(dub_ir,out):
    temp = dub_ir.d - dub_ir.sa + dub_ir.sb
    p_sq = 2 + dub_ir.d_sq - (2*dub_ir.c_ab) + (2*dub_ir.d * (dub_ir.sb-dub_ir.sa))

    if p_sq >= 0:
        temp1 = math.atan2((dub_ir.ca - dub_ir.cb),temp)
        out = (mod2pi(dub_ir.alpha-temp1),
                math.sqrt(p_sq),
                mod2pi(temp1-dub_ir.beta))
        return out
    return -1

def dubins_LSR(dub_ir,out):
    p_sq = -2 + (dub_ir.d_sq) + (2 * dub_ir.c_ab) + (2 * dub_ir.d * (dub_ir.sa + dub_ir.sb))

    if p_sq >= 0:
        p = math.sqrt(p_sq)
        tmp0 = math.atan2( (-dub_ir.ca - dub_ir.cb), (dub_ir.d + dub_ir.sa + dub_ir.sb) ) - math.atan2(-2.0, p)
        out= (mod2pi(tmp0 - dub_ir.alpha),
            p,
            mod2pi(tmp0 - mod2pi(dub_ir.beta)))
        return out
    return -1

def dubins_RSL(dub_ir,out):
    p_sq = -2 + dub_ir.d_sq + (2 * dub_ir.c_ab) - (2 * dub_ir.d * (dub_ir.sa + dub_ir.sb))

    if p_sq >= 0:
        p    = math.sqrt(p_sq)
        tmp0 = math.atan2( (dub_ir.ca + dub_ir.cb), (dub_ir.d - dub_ir.sa - dub_ir.sb) ) - math.atan2(2.0, p)
        out = (mod2pi(dub_ir.alpha - tmp0),
                p,
                mod2pi(dub_ir.beta - tmp0))
        return out
    return -1

def dubins_RLR(dub_ir,out):
    tmp0 = (6. - dub_ir.d_sq + 2*dub_ir.c_ab + 2*dub_ir.d*(dub_ir.sa - dub_ir.sb)) / 8.
    phi  = math.atan2( dub_ir.ca - dub_ir.cb, dub_ir.d - dub_ir.sa + dub_ir.sb )

    if math.fabs(tmp0) <= 1 :
        p = mod2pi((2*math.pi) - math.acos(tmp0) )
        t = mod2pi(dub_ir.alpha - phi + mod2pi(p/2.))
        out = (t,
                p,
                mod2pi(dub_ir.alpha - dub_ir.beta - t + mod2pi(p)))
        return out
    return -1

def dubins_LRL(dub_ir,out):
    tmp0 = (6. - dub_ir.d_sq + 2*dub_ir.c_ab + 2*dub_ir.d*(dub_ir.sb - dub_ir.sa)) / 8.
    phi = math.atan2( dub_ir.ca - dub_ir.cb, dub_ir.d + dub_ir.sa - dub_ir.sb )

    if math.fabs(tmp0) <= 1:
        p = mod2pi( 2*math.pi - math.acos( tmp0) )
        t = mod2pi(-dub_ir.alpha - phi + p/2.)
        out = (t,
                p,
                mod2pi(mod2pi(dub_ir.beta) - dub_ir.alpha -t + mod2pi(p)))
        return out
    return -1


def dubins_word(dub_ir,path_type,out):
    result = -1
    if path_type == dubins_path_type["LSL"]:
        result = dubins_LSL(dub_ir,out)
    elif path_type == dubins_path_type["RSL"]:
        result = dubins_RSL(dub_ir,out)
    elif path_type == dubins_path_type["LSR"]:
        result = dubins_LSR(dub_ir,out)
    elif path_type == dubins_path_type["RSR"]:
        result = dubins_RSR(dub_ir,out)
    elif path_type == dubins_path_type["LRL"]:
        result = dubins_LRL(dub_ir,out)
    elif path_type == dubins_path_type["RLR"]:
        result = dubins_RLR(dub_ir,out)
    else:
        result = -1
    return result
    
# q0 = (0,100, 0)
# q1 = (0,0,0)
# min_turn = 18
# #RSR 
# shortest_path = dubins_shortest_path(q0,q1,min_turn)
# print(shortest_path.params)
# print(shortest_path.length())

# file = open("dubins_path.dubins","w")
# file.truncate(0)
# dubins_path_sample_many(shortest_path,0.1)
# file.close()

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import numpy as np

    q0 = (0,0,math.pi)
    q1 = (10,10,-math.pi/2)
    min_turn = 2
    dubins_path = dubins_shortest_path(q0,q1,min_turn)

    points = dubins_path_sample_many(dubins_path,0.1)
    x = np.array([])
    y = np.array([])
    z = np.array([])
    for point in points:
        x = np.append(x,point[0])
        y = np.append(y,point[1])


    plt.plot(x,y)
    plt.plot(q0[0],q0[1],'ro',markersize=2)
    plt.show()