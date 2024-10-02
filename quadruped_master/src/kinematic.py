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
import math
from geometry_msgs.msg import Pose


rospy.init_node("motion_control")
rate = rospy.Rate(10) # 10hz  
pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
joints_states = JointState()

joints_states.header = Header()
joints_states.header.stamp = rospy.Time.now()
joints_states.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1','front_left_joint2', 'back_left_joint1', 'back_left_joint2', 'back_right_joint1','back_right_joint2']

L = L1 = L2 = float(0.4) # largo de cada eslabon de una pierna

wpose = Pose()
wpose.position.z = 0.6


def start():
    """Set the quadruped to the home position."""
    joint_position_state=[-1,2.2,-1,2.2,-1,2.2,-1,2.2]
    joints_states.position = joint_position_state
    pub.publish(joints_states)
    rospy.sleep(1.00)
    joint_position_state=[0.7592545338404827,1.169485889801056,0.7592545338404827,1.169485889801056,0.7592545338404827,1.169485889801056,0.7592545338404827,1.169485889801056] # stand up principal
    joints_states.position = joint_position_state
    pub.publish(joints_states)

def calculateIK_floor(x: Float = float(0.35), z: Float = float(0.5)):
    q1 = 0 
    q2 = 0 

    # beta  =-acos(((x**2) + (z**2) - 2*(L**2))/(2*(L**2))) # angulo del link1 con respecto al suelo
    # alpha = atan2(z,x) + atan2((sin(beta)), (1+cos(beta)))# angulo del link2 con respecto al link1
    # q1 = -(alpha + beta)
    # q2 = -beta

    # alpha = atan2(-z, x) + acos( sqrt( (x**2) + (z**2) )/( 2*L ))
    # beta  = -(acos(( (2*L**2) - ( (x**2) + (z**2) ) )/(2*L**2 )) - pi)
    # q1 = (alpha + beta)
    # q2 = beta
    
    alpha = math.radians(80)
    beta = math.asin(z / (2 * L)) - alpha
    x = L * (math.cos(alpha + beta))
    q1 = (alpha + beta)
    q2 = -beta

    # alpha = math.radians(80)
    # x = z / math.tan(alpha)
    # beta = math.atan2(z, x) - alpha
    # q1 = (alpha + beta)
    # q2 = -beta
    # print(x)



    if(complex(alpha).imag != 0):
        print("alpha Impossible Angle")
    elif(complex(beta).imag != 0):
        print("beta Impossible Angle")
    elif(complex(q1).imag != 0):
        print("q1 Impossible Angle")
    elif(complex(q2).imag != 0):
        print("q2 Impossible Angle")

    rospy.logwarn("Beta: {}, Alpha: {}, q1: {}, q2: {}, (X Z): {}  {}\n".format(
        str(round(math.degrees(beta),  2)), 
        str(round(math.degrees(alpha), 2)), 
        str(round(math.degrees(q1), 2)), 
        str(round(math.degrees(q2), 2)), 
        str(round(0.4 * (math.cos(q1) + math.cos(q1 + q2)), 2)), 
        str(round(0.4 * (math.sin(q1) + math.sin(q1 + q2)), 2))
    ))

    joint_position_state=[q1,q2,q1,q2,q1,q2,q1,q2] # stand up principal
    joints_states.position = joint_position_state
    pub.publish(joints_states)



if __name__ == "__main__":
    print()
    rospy.sleep(0.5)

    calculateIK_floor(float(0.35), wpose.position.z)
    wpose.position.z -= 0.1
    rospy.sleep(4)

    calculateIK_floor(float(0.35), wpose.position.z)
    wpose.position.z -= 0.1
    rospy.sleep(4)

    calculateIK_floor(float(0.35), wpose.position.z)
    wpose.position.z -= 0.1
    rospy.sleep(4)

    calculateIK_floor(float(0.35), wpose.position.z)
    wpose.position.z -= 0.1
    rospy.sleep(4)

    calculateIK_floor(float(0.35), wpose.position.z)
    wpose.position.z -= 0.1
    rospy.sleep(4)

    calculateIK_floor(float(0.35), wpose.position.z)
    wpose.position.z -= 0.1
    rospy.sleep(4)

    calculateIK_floor(float(0.35), wpose.position.z)
    wpose.position.z -= 0.1
    rospy.sleep(4)

    calculateIK_floor(float(0.35), wpose.position.z)
    wpose.position.z -= 0.1
    rospy.sleep(4)
