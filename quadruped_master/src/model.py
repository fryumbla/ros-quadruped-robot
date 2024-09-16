#!/usr/bin/env python3



from os import MFD_HUGE_512KB
import numpy
from numpy.lib.function_base import angle
from sympy.core.numbers import Pi
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from sympy import *
from time import time
from mpmath import radians
import tf


L0 = 0.3
L = 0.4 # largo de cada eslabon de una pierna

def cal_LIK(x,y,an):
    L1 = L2 = float(L)

    q2 = acos(((x-L0/2*cos(an))*(x-L0/2*cos(an)) + (y+L0/2*sin(an))*(y+L0/2*sin(an)) - L1*L1 - L2*L2)/(2*L1*L2))
    r = atan((L2*sin(q2))/(L1+L2*cos(q2)))
    q1 = atan((y+L0/2*sin(an))/(x-L0/2*cos(an)))-r+an
    if(complex(q1).imag != 0):
        print("Impossible Angle")
        ang = ['c','c']
    elif(complex(q2).imag != 0):
        print("Impossible Angle")
        ang = ['c','c']
    elif(q1<-1.57):
        ang = [3.14+q1,q2]
    elif(q2<-1.57):
        ang = [q1,3.14+q2]

    else:
        ang = [q1, q2]
    return ang