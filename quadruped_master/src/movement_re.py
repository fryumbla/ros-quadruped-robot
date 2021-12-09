#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def main():
    rospy.init_node("motion_control")
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
    joints_states = JointState()

    stand50j14=0.41944732836554044
    stand50j58=1.719784407902978

    stand65j14=0.7592545338404827
    stand65j58=1.169485889801056

    g30=0.5235987756
    g45=0.7853981634

    angle=0
    angle2=0.5235987756

    while not rospy.is_shutdown():

        joints_states.header = Header()
        joints_states.header.stamp = rospy.Time.now()
        joints_states.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1','front_left_joint2', 'back_left_joint1', 'back_left_joint2', 'back_right_joint1','back_right_joint2']
        
        time=1
        walkingtime=0.5
        rate = rospy.Rate(10) # 10hz  

        
        # joint_position_state=[-1,2,-1,2,-1,2,-1,2]
        # joint_position_state=[0,0,0,0,0,0,0,0]
        imp = input ("Enter number: ")
        number = int(imp)
        if (number==1):
            print("hola")
            joint_position_state=[0,0,0,0,0,0,0,0]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
        if (number==2):
            # up
            joint_position_state=[-1,2.2,-1,2.2,-1,2.2,-1,2.2]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58] # stand up principal
            joints_states.position = joint_position_state
            pub.publish(joints_states)
        if (number==3):
            # down
            joint_position_state=[-1,2.2,-1,2.2,-1,2.2,-1,2.2]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-1,0,-1,0,-1,0,-1,0]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[0,0,0,0,0,0,0,0]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
        if (number==4):
            joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
        if (number==5):
            joint_position_state=[stand50j14,stand50j58,stand50j14,stand50j58,stand50j14,stand50j58,stand50j14,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)

        if (number==6):
            #foward 
            joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[stand65j14-(stand65j14-0.6569451124922087),stand65j58+(1.4051585874080312-stand65j58),stand65j14,stand65j58,0.6569451124922087,1.4051585874080312,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[stand65j14-(stand65j14-0.6810971765413939),stand65j58+(1.4051585874080312-stand65j58),stand65j14,stand65j58,0.6810971765413939,1.5199932261951814,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.5199932261951814-stand65j58),stand65j14,stand65j58,0.8063266033079631,1.4553773075593721,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.4553773075593721-stand65j58),stand65j14,stand65j58,0.948427838239876,1.2447369771100414,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            # joint_position_state=[stand65j14+(0.948427838239876-stand65j14),stand65j58-(1.2447369771100414-stand65j58),stand65j14,stand65j58,0.948427838239876,1.2447369771100414,stand65j14,stand65j58]
            # joints_states.position = joint_position_state
            # pub.publish(joints_states)
            # rospy.sleep(walkingtime)
            joint_position_state=[stand65j14,stand65j58,stand65j14-(stand65j14-0.6569451124922087),stand65j58+(1.4051585874080312-stand65j58),stand65j14,stand65j58,0.6569451124922087,1.4051585874080312]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[stand65j14,stand65j58,stand65j14-(stand65j14-0.6810971765413939),stand65j58+(1.4051585874080312-stand65j58),stand65j14,stand65j58,0.6810971765413939,1.5199932261951814]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[stand65j14,stand65j58,stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.5199932261951814-stand65j58),stand65j14,stand65j58,0.8063266033079631,1.4553773075593721]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[stand65j14,stand65j58,stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.4553773075593721-stand65j58),stand65j14,stand65j58,0.948427838239876,1.2447369771100414]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)

        if (number==7):
            # backward right
            joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[0.6569451124922087,1.4051585874080312,stand65j14,stand65j58,stand65j14-(stand65j14-0.6569451124922087),stand65j58+(1.4051585874080312-stand65j58),stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[0.6810971765413939,1.5199932261951814,stand65j14,stand65j58,stand65j14-(stand65j14-0.6810971765413939),stand65j58+(1.4051585874080312-stand65j58),stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[0.8063266033079631,1.4553773075593721,stand65j14,stand65j58,stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.5199932261951814-stand65j58),stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[0.948427838239876,1.2447369771100414,stand65j14,stand65j58,stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.4553773075593721-stand65j58),stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            #left
            joint_position_state=[stand65j14,stand65j58,0.6569451124922087,1.4051585874080312,stand65j14,stand65j58,stand65j14-(stand65j14-0.6569451124922087),stand65j58+(1.4051585874080312-stand65j58)]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[stand65j14,stand65j58,0.6810971765413939,1.5199932261951814,stand65j14,stand65j58,stand65j14-(stand65j14-0.6810971765413939),stand65j58+(1.4051585874080312-stand65j58)]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[stand65j14,stand65j58,0.8063266033079631,1.4553773075593721,stand65j14,stand65j58,stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.5199932261951814-stand65j58)]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[stand65j14,stand65j58,0.948427838239876,1.2447369771100414,stand65j14,stand65j58,stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.4553773075593721-stand65j58)]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)
            joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(walkingtime)

        if (number==8):
            # rotation
            joint_position_state=[stand50j14,stand50j58,stand50j14,stand50j58,stand50j14,stand50j58,stand50j14,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[stand50j14+angle,stand50j58-angle,stand50j14+angle,stand50j58-angle,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398,2.6179,stand50j14+angle,stand50j58-angle,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398,0,stand50j14+angle,stand50j58-angle,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398+g30*4/3,stand50j58-angle-g30*2,stand50j14+angle,stand50j58-angle,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            # joint_position_state=[-0.785398,stand50j58-angle,stand50j14+angle,stand50j58-angle,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            # joints_states.position = joint_position_state
            # pub.publish(joints_states)
            # rospy.sleep(time)
            # joint_position_state=[0,stand50j58-angle,stand50j14+angle,stand50j58-angle,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            # joints_states.position = joint_position_state
            # pub.publish(joints_states)
            # rospy.sleep(time)
            # joint_position_state=[stand50j14,stand50j58-angle*2,stand50j14+angle,stand50j58-angle,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            # joints_states.position = joint_position_state
            # pub.publish(joints_states)
            # rospy.sleep(time)

            # #pierna frente izquierda
            joint_position_state=[-0.785398+g30*4/3,stand50j58-angle-g30*2,-0.785398,2.6179,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398+g30*4/3,stand50j58-angle-g30*2,-0.785398,0,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398+g30*4/3,stand50j58-angle-g30*2,-0.785398+g30*4/3,stand50j58-angle-g30*2,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            # joint_position_state=[stand50j14,stand50j58-angle*2,-0.785398,2.6179,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            # joints_states.position = joint_position_state
            # pub.publish(joints_states)
            # rospy.sleep(time)
            # joint_position_state=[stand50j14,stand50j58-angle*2,-0.785398,stand50j58-angle,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            # joints_states.position = joint_position_state
            # pub.publish(joints_states)
            # rospy.sleep(time)
            # joint_position_state=[stand50j14,stand50j58-angle*2,0,stand50j58-angle,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            # joints_states.position = joint_position_state
            # pub.publish(joints_states)
            # rospy.sleep(time)
            # joint_position_state=[stand50j14,stand50j58-angle*2,stand50j14,stand50j58-angle*2,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            # joints_states.position = joint_position_state
            # pub.publish(joints_states)
            # rospy.sleep(time)


            joint_position_state=[-0.785398+g30*4/3,stand50j58-angle-g30*2,-0.785398+g30*4/3,stand50j58-angle-g30*2,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398+g30*4/3-angle2,stand50j58-angle-g30*2+angle2,-0.785398+g30*4/3-angle2,stand50j58-angle-g30*2+angle2,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398+g30*4/3-angle2, stand50j58-angle-g30*2+angle2*2, -0.785398+g30*4/3-angle2, stand50j58-angle-g30*2+angle2*2, stand50j14-angle,stand50j58-angle-angle2,stand50j14-angle,stand50j58-angle-angle2]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398+g30*4/3-angle2, stand50j58-angle-(g30/2)*2+angle2*2, -0.785398+g30*4/3-angle2, stand50j58-angle-(g30/2)*2+angle2*2, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            
            joint_position_state=[-g30*2,2.6179, -0.785398+g30*4/3-angle2, stand50j58-angle-(g30/2)*2+angle2*2, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-g30*2,g30*3, -0.785398+g30*4/3-angle2, stand50j58-angle-(g30/2)*2+angle2*2, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            # joint_position_state=[-g30,g30*3, -0.785398+g30*4/3-angle2, stand50j58-angle-(g30/2)*2+angle2*2, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            # joints_states.position = joint_position_state
            # pub.publish(joints_states)
            # rospy.sleep(time)
            joint_position_state=[-0.785398,g30*3, -0.785398+g30*4/3-angle2, stand50j58-angle-(g30/2)*2+angle2*2, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
        
            #frontal izquierda
            joint_position_state=[-0.785398,g30*3,-g30*2,2.6179, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398,g30*3,-g30*2,g30*3, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398,g30*3,-0.785398,g30*3, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)

            

            # #trasera derecha
            joint_position_state=[-0.785398,g30*3,-0.785398,g30*3, stand50j14-angle,stand50j58-angle-angle2-g30,-g30/2,stand50j58-angle-angle2-g30]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398,g30*3,-0.785398,g30*3, stand50j14-angle,stand50j58-angle-angle2-g30,-g30/2,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398,g30*3,-0.785398,g30*3, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)

            # #trasera izquierda
            joint_position_state=[-0.785398,g30*3,-0.785398,g30*3, -g30/2,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398,g30*3,-0.785398,g30*3, -g30/2,stand50j58,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398,g30*3,-0.785398,g30*3, stand50j14-angle,stand50j58,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)

            # Body motion 
            joint_position_state=[-0.785398-(g45/2),g30*3+g45,-0.785398-(g45/2),g30*3+g45, stand50j14-angle,stand50j58-g45,stand50j14-angle,stand50j58-g45]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)

            # #trasera derecha
            joint_position_state=[-0.785398-(g45/2),g30*3+g45,-0.785398-(g45/2),g30*3+g45, stand50j14-angle,stand50j58-g45,-g30/2,stand50j58-angle-angle2-g30]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398-(g45/2),g30*3+g45,-0.785398-(g45/2),g30*3+g45, stand50j14-angle,stand50j58-g45,-g30/3,stand50j58-angle+g45/2]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398-(g45/2),g30*3+g45,-0.785398-(g45/2),g30*3+g45, stand50j14-angle,stand50j58-g45, g30/3,stand50j58-angle+g45/2]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)

            #trasera izquierda
            joint_position_state=[-0.785398-(g45/2),g30*3+g45,-0.785398-(g45/2),g30*3+g45, -g30/2,stand50j58-angle-angle2-g30, g30/3,stand50j58-angle+g45/2]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398-(g45/2),g30*3+g45,-0.785398-(g45/2),g30*3+g45, -g30/3,stand50j58-angle+g45/2, g30/3,stand50j58-angle+g45/2]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398-(g45/2),g30*3+g45,-0.785398-(g45/2),g30*3+g45, g30/3,stand50j58-angle+g45/2, g30/3,stand50j58-angle+g45/2]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)

            # Body motion 
            joint_position_state=[-g30,g30*5,-g30,g30*5, stand50j14-angle,stand50j58,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)

            # frontal derecha
            joint_position_state=[-g30,g30*5,-g30,g30*5, stand50j14-angle,stand50j58,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398,g30*5,-g30,g30*5, stand50j14-angle,stand50j58,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398,g30*4,-g30,g30*5, stand50j14-angle,stand50j58,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398,g30*3,-g30,g30*5, stand50j14-angle,stand50j58,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-g30,g30*3,-g30,g30*5, stand50j14-angle,stand50j58,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)

            # frontal izquierda
            joint_position_state=[-g30,g30*3,-0.785398,g30*5, stand50j14-angle,stand50j58,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-g30,g30*3,-0.785398,g30*4, stand50j14-angle,stand50j58,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-g30,g30*3,-0.785398,g30*3, stand50j14-angle,stand50j58,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-g30,g30*3,-g30,g30*3, stand50j14-angle,stand50j58,stand50j14-angle,stand50j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)

            # Body motion 
            joint_position_state=[g30/2,g30*3,g30/2,g30*3, stand50j14-angle+g30/2,stand50j58/2,stand50j14-angle+g30/2,stand50j58/2]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[g30/3,g30*4,g30/3,g30*4, stand50j14-angle+g30/3,0,stand50j14-angle+g30/3,0]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)


        if (number==9):
            # segundo paso
            joint_position_state=[g30/3,g30*4,g30/3,g30*4, stand50j14-angle+g30/3,0,stand50j14-angle-g45,g30*4]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            

            

                    


if __name__ == '__main__':
    main()