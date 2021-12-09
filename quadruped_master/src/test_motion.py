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
L = 0.4
Lgap = 0.7
#IK calculation
def cal_LIK(x,y,an):
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

def cal_RIK(x,y,an):
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


def main():
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
    while not rospy.is_shutdown():

        joints_states.header = Header()
        joints_states.header.stamp = rospy.Time.now()
        joints_states.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1','front_left_joint2', 'back_left_joint1', 'back_left_joint2', 'back_right_joint1','back_right_joint2']
        time = 0.05
        walkingtime=0.5
        rate = rospy.Rate(10) # 10hz  
        
        inp = input ("Enter Movement : ")
        
        # start

        if ( inp == 'on'):
            print("Start")
            if(joint_position_state == [0,0,0,0,0,0,0,0]):
                mAll = []
                x = 0.35; y = 0.6; an = 0
                Q1 = cal_LIK(x,y,an)[0]
                # print(Q1)
                Q2 = cal_LIK(x,y,an)[1]
                # print(Q2)
                Q5 = cal_RIK(x,y,an)[0]
                # print(Q1)
                Q6 = cal_RIK(x,y,an)[1]
                # print(Q2)
                Q3 = Q1
                Q4 = Q2
                Q7 = Q5
                Q8 = Q6
                m1 = angList(0,Q1); m2 = angList(0,Q2); m3 = angList(0,Q3); m4 = angList(0,Q4); m5 = angList(0,Q5); m6 = angList(0,Q6); m7 = angList(0,Q7); m8 = angList(0,Q8)
                mAll = [m1,m2,m3,m4,m5,m6,m7,m8]   
                for s in range(0,len(m1)-1):
                    joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                x = 0.35; y = 0.6 ;bx = x; by = y
                print("current position is (%.2f, %.2f)" %(x,y))
                
            
        # finish
        if (inp == 'off'):
            print("End")
            if(joint_position_state != [0,0,0,0,0,0,0,0]):
                mAll = []
                m1 = angList(bQ1,0); m2 = angList(bQ2,0); m3 = angList(bQ3,0); m4 = angList(bQ4,0); m5 = angList(bQ5,0); m6 = angList(bQ6,0); m7 = angList(bQ7,0); m8 = angList(bQ8,0)
                mAll = [m1,m2,m3,m4,m5,m6,m7,m8]                
                for s in range(0,len(m1)-1):
                    joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                joint_position_state=[0,0,0,0,0,0,0,0]
                joints_states.position = joint_position_state
                pub.publish(joints_states)
                Q1 = Q2 = Q3 = Q4 = Q5 = Q6 = Q7 = Q8 = 0
                bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                x = 0.35; y = 0; bx = x; by = y; an = 0

        if (inp == 'move xy'):
            x = float(input("Enter x position : "))
            y = float(input("Enter y position : "))
            print("Move to (%.2f,%.2f)" %(x,y))
            mAll = []
            Q1 = cal_LIK(x,y,an)[0]            
            #print(Q1)
            Q2 = cal_LIK(x,y,an)[1]            
            #print(Q2)            
            Q5 = cal_RIK(x,y,an)[0]
            #print(Q5)
            Q6 = cal_RIK(x,y,an)[1]            
            #print(Q6)
            if(Q1 == 'c' or Q2 == 'c' or Q5 == 'c' or Q6 == 'c'):
                Q1 = bQ1
                Q2 = bQ2
                Q5 = bQ5
                Q6 = bQ6
                x = bx; y = by
            Q3 = Q1
            Q4 = Q2
            Q7 = Q5
            Q8 = Q6
            m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
            mAll = [m1,m2,m3,m4,m5,m6,m7,m8]   
            for s in range(0,len(m1)-1):
                joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                joints_states.position = joint_position_state
                pub.publish(joints_states)
                rospy.sleep(time)
            bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
            print("current position is (%.2f, %.2f)" %(x,y))
    #up
        if (inp == 'w'):
            print("Move up 0.1(step)")
            y +=0.1
            mAll = []
            Q1 = cal_LIK(x,y,an)[0]            
            #print(Q1)
            Q2 = cal_LIK(x,y,an)[1]            
            #print(Q2)
            Q5 = cal_RIK(x,y,an)[0]            
            #print(Q5)
            Q6 = cal_RIK(x,y,an)[1]            
            #print(Q6)
            if(Q1 == 'c' or Q2 == 'c' or Q5 == 'c' or Q6 == 'c'):
                Q1 = bQ1
                Q2 = bQ2
                Q5 = bQ5
                Q6 = bQ6
                y-=0.1
            Q3 = Q1
            Q4 = Q2
            Q7 = Q5
            Q8 = Q6
            m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
            mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
            for s in range(0,len(m1)-1):
                joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                joints_states.position = joint_position_state
                pub.publish(joints_states)
                rospy.sleep(time)
            bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
            bx = x; by = y
            print("current position is (%.2f, %.2f)" %(x,y))
    #down
        if (inp == 's'):
            print("Move down 0.1(step)")
            y-=0.1         
            mAll = []
            Q1 = cal_LIK(x,y,an)[0]
            #print(Q1)
            Q2 = cal_LIK(x,y,an)[1]
            #print(Q2)            
            Q5 = cal_RIK(x,y,an)[0]            
            #print(Q5)
            Q6 = cal_RIK(x,y,an)[1]            
            #print(Q6)
            if(Q1 == 'c' or Q2 == 'c' or Q5 == 'c' or Q6 == 'c' or y<0.1):
                Q1 = bQ1
                Q2 = bQ2
                Q5 = bQ5
                Q6 = bQ6
                y+=0.1
            Q3 = Q1
            Q4 = Q2
            Q7 = Q5
            Q8 = Q6
            m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
            mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
            
            for s in range(0,len(m1)-1):
                joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                joints_states.position = joint_position_state
                pub.publish(joints_states)
                rospy.sleep(time)
            bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
            bx = x; by = y
            print("current position is (%.2f, %.2f)" %(x,y))
    #left
        if (inp == 'a'):
            print("Move left 0.1(step)")
            x+=0.1           
            mAll = []
            Q1 = cal_LIK(x,y,an)[0]            
            #print(Q1)
            Q2 = cal_LIK(x,y,an)[1]            
            #print(Q2)            
            Q5 = cal_RIK(x,y,an)[0]
            #print(Q5)
            Q6 = cal_RIK(x,y,an)[1]            
            #print(Q6)
            if(Q1 == 'c' or Q2 == 'c' or Q5 == 'c' or Q6 == 'c' or x>0.7):
                Q1 = bQ1
                Q2 = bQ2
                Q5 = bQ5
                Q6 = bQ6
                x-=0.1
            Q3 = Q1
            Q4 = Q2
            Q7 = Q5
            Q8 = Q6
            m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
            mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
            for s in range(0,len(m1)-1):
                joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                joints_states.position = joint_position_state
                pub.publish(joints_states)
                rospy.sleep(time)
            bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8 
            bx = x; by = y
            print("current position is (%.2f, %.2f)" %(x,y))
        #right
        if (inp == 'd'):
            print("Move right 0.1(step)")
            x-=0.1            
            mAll = []
            Q1 = cal_LIK(x,y,an)[0]            
            #print(Q1)
            Q2 = cal_LIK(x,y,an)[1]            
            #print(Q2)            
            Q5 = cal_RIK(x,y,an)[0]
            #print(Q5)
            Q6 = cal_RIK(x,y,an)[1]            
            #print(Q6)
            if(Q1 == 'c' or Q2 == 'c' or Q5 == 'c' or Q6 == 'c' or x<0.05):
                Q1 = bQ1
                Q2 = bQ2
                Q5 = bQ5
                Q6 = bQ6
                x+= 0.1
            Q3 = Q1
            Q4 = Q2
            Q7 = Q5
            Q8 = Q6
            m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
            mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
            for s in range(0,len(m1)-1):
                joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                joints_states.position = joint_position_state
                pub.publish(joints_states)
                rospy.sleep(time)
            bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
            print("current position is (%.2f, %.2f)" %(x,y))
            bx = x; by = y
        #current Postion
        if(inp == 'pos'):
            print("x = %.2f y = %.2f angle : %.2f" %(x,y,an))
        
        if(inp == 'set'):
            print("Standard Position")
            x = 0.35
            y = 0.6
            an = 0
            mAll = []
            Q1 = cal_LIK(x,y,an)[0]
            # print(Q1)
            Q2 = cal_LIK(x,y,an)[1]
            # print(Q2)
            Q5 = cal_RIK(x,y,an)[0]
            # print(Q1)
            Q6 = cal_RIK(x,y,an)[1]
            # print(Q2)
            Q3 = Q1
            Q4 = Q2
            Q7 = Q5
            Q8 = Q6
            m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
            mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
            for s in range(0,len(m1)-1):
                joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                joints_states.position = joint_position_state
                pub.publish(joints_states)
                rospy.sleep(time)
            bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
            
            bx = x; by = y
            print("current position is (%.2f, %.2f)" %(x,y))

        if(inp == 'move'):
            d = 0.15
            h = 0.05
            while(1):
                fb = input("Direction : ")
                if(fb == 'd'):
                    mAll = []
                    # move leg 1
                    Q3 = cal_LIK(x-d,y,an)[0]; Q4 = cal_LIK(x-d,y,an)[1]; Q5 = cal_RIK(x-d,y,an)[0]; Q6 = cal_RIK(x-d,y,an)[1]
                    if(Q3 == 'c' or Q4 == 'c' or Q5 == 'c' or Q6 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    Q1 = cal_LIK(x,y-h,an)[0]; Q2 = cal_LIK(x,y-h,an)[1]
                    if(Q1 == 'c' or Q2 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    Q7 = Q5
                    Q8 = Q6
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q1 = cal_LIK(x,y,an)[0]; Q2 = cal_LIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    # move leg 2
                    Q3 = cal_LIK(x,y-h,an)[0]; Q4 = cal_LIK(x,y-h,an)[1]
                    if(Q3 == 'c' or Q4 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q3 = cal_LIK(x,y,an)[0]; Q4 = cal_LIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    # move leg 3
                    Q5 = cal_RIK(x,y-h,an)[0]; Q6 = cal_RIK(x,y-h,an)[1]
                    if(Q5 == 'c' or Q6 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q5 = cal_RIK(x,y,an)[0]; Q6 = cal_RIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    # move leg 4
                    Q7 = cal_RIK(x,y-h,an)[0]; Q8 = cal_RIK(x,y-h,an)[1]
                    if(Q7 == 'c' or Q8 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q7 = cal_RIK(x,y,an)[0]; Q8 = cal_RIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    x = bx; y = by
                    print("Move right %.2f(step)" %d)

                elif(fb == 'a'):
                    mAll = []
                    # move leg 3
                    Q1 = cal_LIK(x+d,y,an)[0]; Q2 = cal_LIK(x+d,y,an)[1]; Q7 = cal_RIK(x+d,y,an)[0]; Q8 = cal_RIK(x+d,y,an)[1]
                    if(Q1 == 'c' or Q2 == 'c' or Q7 == 'c' or Q8 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    Q5 = cal_RIK(x,y-h,an)[0]; Q6 = cal_RIK(x,y-h,an)[1]
                    if(Q5 == 'c' or Q6 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    Q3 = Q1
                    Q4 = Q2
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q5 = cal_RIK(x,y,an)[0]; Q6 = cal_RIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    # move leg 4
                    Q7 = cal_RIK(x,y-h,an)[0]; Q8 = cal_RIK(x,y-h,an)[1]
                    if(Q7 == 'c' or Q8 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q7 = cal_RIK(x,y,an)[0]; Q8 = cal_RIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    # move leg 4
                    Q3 = cal_LIK(x,y-h,an)[0]; Q4 = cal_LIK(x,y-h,an)[1]
                    if(Q3 == 'c' or Q8 == 'c' ):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q3 = cal_LIK(x,y,an)[0]; Q4 = cal_LIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    # move leg 1
                    Q1 = cal_LIK(x,y-h,an)[0]; Q2 = cal_LIK(x,y-h,an)[1]
                    if(Q1 == 'c' or Q2 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q1 = cal_LIK(x,y,an)[0]; Q2 = cal_LIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    x = bx; y = by
                    print("Move right %.2f(step)" %d)
                    

                elif(fb == 'stop'):
                    break
        #turn angles
        if (inp == 'angle'):
            da = 0.2618; 
            #dx = L0/2*cos(a); dy = L0/2*sin(a)
            while(1):
                ud = input("Direction : ")
                if(ud == 'w'):
                    mAll = []
                    an+=da
                    Q1 = cal_LIK(x,y,an)[0]            
                    #print(Q1)
                    Q2 = cal_LIK(x,y,an)[1]            
                    #print(Q2)            
                    Q5 = cal_RIK(x,y,an)[0]
                    #print(Q5)
                    Q6 = cal_RIK(x,y,an)[1]            
                    #print(Q6)
                    if(Q1 == 'c' or Q2 == 'c' or Q5 == 'c' or Q6 == 'c'):
                        Q1 = bQ1
                        Q2 = bQ2
                        Q5 = bQ5
                        Q6 = bQ6
                        an-=da
                    Q3 = Q1
                    Q4 = Q2
                    Q7 = Q5
                    Q8 = Q6
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    print("angle change up")
                    bx = x; by = y; bAn = an

                elif(ud == 's'):
                    mAll = []
                    an-=da
                    Q1 = cal_LIK(x,y,an)[0]            
                    #print(Q1)
                    Q2 = cal_LIK(x,y,an)[1]            
                    #print(Q2)            
                    Q5 = cal_RIK(x,y,an)[0]
                    #print(Q5)
                    Q6 = cal_RIK(x,y,an)[1]            
                    #print(Q6)
                    if(Q1 == 'c' or Q2 == 'c' or Q5 == 'c' or Q6 == 'c'):
                        Q1 = bQ1
                        Q2 = bQ2
                        Q5 = bQ5
                        Q6 = bQ6
                        an+=da
                    Q3 = Q1
                    Q4 = Q2
                    Q7 = Q5
                    Q8 = Q6
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    print("angle change down")
                    bx = x; by = y; bAn = an
                elif(ud == 'stop'):
                    break
        #steps
        if(inp == 'step'):
            d = 0.15
            h = 0.05
            while(1):
                fb = input("Direction : ")
                if(fb == 'up'):
                    mAll = []
                    # move leg 1
                    Q3 = cal_LIK(x-d,y,an)[0]; Q4 = cal_LIK(x-d,y,an)[1]; Q5 = cal_RIK(x-d,y,an)[0]; Q6 = cal_RIK(x-d,y,an)[1]
                    if(Q3 == 'c' or Q4 == 'c' or Q5 == 'c' or Q6 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    Q1 = cal_LIK(x,y-h,an)[0]; Q2 = cal_LIK(x,y-h,an)[1]
                    if(Q1 == 'c' or Q2 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    Q7 = Q5
                    Q8 = Q6
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q1 = cal_LIK(x,y,an)[0]; Q2 = cal_LIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    # move leg 2
                    Q3 = cal_LIK(x,y-h,an)[0]; Q4 = cal_LIK(x,y-h,an)[1]
                    if(Q3 == 'c' or Q4 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q3 = cal_LIK(x,y,an)[0]; Q4 = cal_LIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    # move leg 3
                    Q5 = cal_RIK(x,y-h,an)[0]; Q6 = cal_RIK(x,y-h,an)[1]
                    if(Q5 == 'c' or Q6 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q5 = cal_RIK(x,y,an)[0]; Q6 = cal_RIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    # move leg 4
                    Q7 = cal_RIK(x,y-h,an)[0]; Q8 = cal_RIK(x,y-h,an)[1]
                    if(Q7 == 'c' or Q8 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q7 = cal_RIK(x,y,an)[0]; Q8 = cal_RIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    x = bx; y = by
                    print("Move right %.2f(step)" %d)

                elif(fb == 'down'):
                    mAll = []
                    # move leg 3
                    Q1 = cal_LIK(x+d,y,an)[0]; Q2 = cal_LIK(x+d,y,an)[1]; Q7 = cal_RIK(x+d,y,an)[0]; Q8 = cal_RIK(x+d,y,an)[1]
                    if(Q1 == 'c' or Q2 == 'c' or Q7 == 'c' or Q8 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    Q5 = cal_RIK(x,y-h,an)[0]; Q6 = cal_RIK(x,y-h,an)[1]
                    if(Q5 == 'c' or Q6 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    Q3 = Q1
                    Q4 = Q2
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q5 = cal_RIK(x,y,an)[0]; Q6 = cal_RIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    # move leg 4
                    Q7 = cal_RIK(x,y-h,an)[0]; Q8 = cal_RIK(x,y-h,an)[1]
                    if(Q7 == 'c' or Q8 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q7 = cal_RIK(x,y,an)[0]; Q8 = cal_RIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    # move leg 4
                    Q3 = cal_LIK(x,y-h,an)[0]; Q4 = cal_LIK(x,y-h,an)[1]
                    if(Q3 == 'c' or Q8 == 'c' ):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q3 = cal_LIK(x,y,an)[0]; Q4 = cal_LIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    # move leg 1
                    Q1 = cal_LIK(x,y-h,an)[0]; Q2 = cal_LIK(x,y-h,an)[1]
                    if(Q1 == 'c' or Q2 == 'c'):
                        Q1 = bQ1; Q2 = bQ2; Q3 = bQ3; Q4 = bQ4; Q5 = bQ5; Q6 = bQ6; Q7 = bQ7; Q8 = bQ8
                    m1 = angList(bQ1,Q1); m2 = angList(bQ2,Q2); m3 = angList(bQ3,Q3); m4 = angList(bQ4,Q4); m5 = angList(bQ5,Q5); m6 = angList(bQ6,Q6); m7 = angList(bQ7,Q7); m8 = angList(bQ8,Q8)
                    mAll = [m1,m2,m3,m4,m5,m6,m7,m8]  
                    for s in range(0,len(m1)-1):
                        joint_position_state = [mAll[0][s],mAll[1][s],mAll[2][s],mAll[3][s],mAll[4][s],mAll[5][s],mAll[6][s],mAll[7][s]]
                        joints_states.position = joint_position_state
                        pub.publish(joints_states)
                        rospy.sleep(time)
                    Q1 = cal_LIK(x,y,an)[0]; Q2 = cal_LIK(x,y,an)[1]
                    joint_position_state = [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8]
                    joints_states.position = joint_position_state
                    pub.publish(joints_states)
                    rospy.sleep(time)
                    bQ1 = Q1; bQ2 = Q2; bQ3 = Q3; bQ4 = Q4; bQ5 = Q5; bQ6 = Q6; bQ7 = Q7; bQ8 = Q8
                    x = bx; y = by
                    print("Move right %.2f(step)" %d)
                    

                elif(fb == 'stop'):
                    break
                    
                       
            
            


if __name__ == '__main__':
    main()