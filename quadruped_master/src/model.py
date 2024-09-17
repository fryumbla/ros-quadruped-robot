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

def angList(q1,q2):
    h = (float(q2)-float(q1))/30
    Leg_angles = []
    if h == 0:
        for i in range (0,30):
            Leg_angles.append(q1)     
    else:
        Leg_angles = numpy.arange(q1,q2,h)
    return Leg_angles

L0 = 0.3
L = 0.4 # largo de cada eslabon de una pierna
Lgap = 0.7

#IK calculation
def cal_LIK(x,y,an):    # q1 and q2 for left leg
    L1 = L2 = float(L)
    # x-L0/2 = L1*cos(q1)+L2*cos(q1+q2)
    # y = L1*sin(q1)+L2*sin(q1+q2)

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
    #    ang = [-1.57-q1,q2]
    elif(q2<-1.57):
        ang = [q1,3.14+q2]
    #    ang = [q1,-1.57-q2]
    #elif(q1>1.57):
    #    ang = [q1-1.57,q2]
    #elif(q2>1.57):
    #    ang = [q1,q2-1.57]
    else:
        ang = [q1, q2]
    return ang

def cal_RIK(x,y,an):    # q1 and q2 for right leg
    L3 = L4 = float(L)
    # x+L0/2-0.7 = L1*cos(q1)+L2*cos(q1+q2)
    # y = L1*sin(q1)+L2*sin(q1+q2)
    q2 = acos(((Lgap-x-L0/2*cos(an))*(Lgap-x-L0/2*cos(an)) + (y-L0/2*sin(an))*(y-L0/2*sin(an)) - L3*L3 - L4*L4)/(2*L3*L4))
    r = atan((L4*sin(q2))/(L3+L4*cos(q2)))
    q1 = atan((y-L0/2*sin(an))/(Lgap-x-L0/2*cos(an)))-r-an
    if(complex(q1).imag != 0):
        print("Impossible Angle")
        ang = ['c','c']
    elif(complex(q2).imag != 0):
        print("Impossible Angle")
        ang = ['c','c']    
    elif(q1<-1.57):
        ang = [3.14+q1,q2]
    #    ang = [-1.57-q1,q2]
    elif(q2<-1.57):
        ang = [q1,3.14+q2]
    #    ang = [q1,-1.57-q2]   
    #elif(q1>1.57):
    #    ang = [q1,q2]
    #elif(q2>1.57):
    #    ang = [q1,q2]
    else:
        ang = [q1, q2]
    return ang

rospy.init_node("motion_control")
pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
joints_states = JointState()

# stand position angles
#stand50j14=0.41944732836554044
#stand50j58=1.719784407902978

#stand65j14=0.7592545338404827
#stand65j58=1.169485889801056

#g30=0.5235987756
#g45=0.7853981634

#angle=0
#angle2=0.5235987756
# joint_postion_state [frnt right up, frint right down,front_left_joint up,front_left_joint down, back_left_joint up, back_left_joint down, back_right_joint up,back_right_joint down]
joint_position_state=[0,0,0,0,0,0,0,0]
bQ1 = 0; bQ2 = 0; bQ3 = 0; bQ4 = 0; bQ5 = 0; bQ6 = 0; bQ7 = 0; bQ8 = 0
Q1 = 0; Q2 = 0; Q3 = 0; Q4 = 0; Q5 = 0; Q6 = 0; Q7 = 0; Q8 = 0
x = 0.35; y = 0; bx = x ; by = y
an = 0; bAn = an; 

joints_states.header = Header()
joints_states.header.stamp = rospy.Time.now()
joints_states.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1','front_left_joint2', 'back_left_joint1', 'back_left_joint2', 'back_right_joint1','back_right_joint2']
time = 0.05
walkingtime=0.5
rate = rospy.Rate(10) # 10hz  

if(joint_position_state == [0,0,0,0,0,0,0,0]):
    mAll = []
    x = 0.6; y = 0.5; an = 0

    Q1 = cal_LIK(x,y,an)[0]
    print(Q1)
    
    Q2 = cal_LIK(x,y,an)[1]
    print(Q2)
    
    Q5 = cal_RIK(x,y,an)[0]
    print(Q5)
    
    Q6 = cal_RIK(x,y,an)[1]
    print(Q6)
    
    Q3 = Q1
    Q4 = Q2
    Q7 = Q5
    Q8 = Q6

    m1 = angList(0,Q1); m2 = angList(0,Q2); m3 = angList(0,Q3); m4 = angList(0,Q4); m5 = angList(0,Q5); m6 = angList(0,Q6); m7 = angList(0,Q7); m8 = angList(0,Q8)
    
    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]   
    for s in range(0,len(m1)-1):
        joint_position_state = [mAll[0][s],mAll[1][s], mAll[2][s], mAll[3][s],mAll[4][s],mAll[5][s], mAll[6][s], mAll[7][s]]
        joints_states.position = joint_position_state
        pub.publish(joints_states)
        rospy.sleep(time)
    
    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
    
    x = 0.35; y = 0.6 ;bx = x; by = y
    
    print("Position for LFK: (%.2f, %.2f)\nPosition for RFK: (%.2f, %.2f)" %(L*(cos(Q1) + cos(Q2)),L*(sin(Q1) + sin(Q2)), L*(cos(Q5) + cos(Q6)),L*(sin(Q5) + sin(Q6))))