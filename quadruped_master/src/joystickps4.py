#!/usr/bin/env python
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
from ds4_driver.msg import Status
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_msgs.msg import Header

class Key2Vel:

    def key_cb(self, key_msg):
        self.joints_states = JointState()
        self.joints_states.header = Header()
        self.joints_states.name = "quadruped_joints"
        self.joints_states.header.stamp = rospy.Time.now()
        self.joints_states.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1','front_left_joint2', 'back_left_joint1', 'back_left_joint2', 'back_right_joint1','back_right_joint2']
        
        stand50j14=0.41944732836554044
        stand50j58=1.719784407902978

        stand65j14=0.7592545338404827
        stand65j58=1.169485889801056

        g30=0.5235987756
        g45=0.7853981634    

        angle=0
        angle2=0.5235987756

        time=5
        walkingtime=2

        

        if key_msg.button_dpad_up == 1: #W, X UP
            self.joint_position_state=[-1,2.2,-1,2.2,-1,2.2,-1,2.2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58] # stand up principal
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            print('up')

        elif key_msg.button_dpad_down == 1:   #S, X DOWN
            self.joint_position_state=[-1,2.2,-1,2.2,-1,2.2,-1,2.2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-1,0,-1,0,-1,0,-1,0]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[0,0,0,0,0,0,0,0]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            print('down')

        elif key_msg.button_dpad_left == 1:   #A, left
            # backward right
            self.joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[0.6569451124922087,1.4051585874080312,stand65j14,stand65j58,stand65j14-(stand65j14-0.6569451124922087),stand65j58+(1.4051585874080312-stand65j58),stand65j14,stand65j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[0.6810971765413939,1.5199932261951814,stand65j14,stand65j58,stand65j14-(stand65j14-0.6810971765413939),stand65j58+(1.4051585874080312-stand65j58),stand65j14,stand65j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[0.8063266033079631,1.4553773075593721,stand65j14,stand65j58,stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.5199932261951814-stand65j58),stand65j14,stand65j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[0.948427838239876,1.2447369771100414,stand65j14,stand65j58,stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.4553773075593721-stand65j58),stand65j14,stand65j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            #left
            self.joint_position_state=[stand65j14,stand65j58,0.6569451124922087,1.4051585874080312,stand65j14,stand65j58,stand65j14-(stand65j14-0.6569451124922087),stand65j58+(1.4051585874080312-stand65j58)]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[stand65j14,stand65j58,0.6810971765413939,1.5199932261951814,stand65j14,stand65j58,stand65j14-(stand65j14-0.6810971765413939),stand65j58+(1.4051585874080312-stand65j58)]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[stand65j14,stand65j58,0.8063266033079631,1.4553773075593721,stand65j14,stand65j58,stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.5199932261951814-stand65j58)]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[stand65j14,stand65j58,0.948427838239876,1.2447369771100414,stand65j14,stand65j58,stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.4553773075593721-stand65j58)]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            print('left')

        elif key_msg.button_dpad_right == 1:   #D, right
            self.joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[stand65j14-(stand65j14-0.6569451124922087),stand65j58+(1.4051585874080312-stand65j58),stand65j14,stand65j58,0.6569451124922087,1.4051585874080312,stand65j14,stand65j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[stand65j14-(stand65j14-0.6810971765413939),stand65j58+(1.4051585874080312-stand65j58),stand65j14,stand65j58,0.6810971765413939,1.5199932261951814,stand65j14,stand65j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.5199932261951814-stand65j58),stand65j14,stand65j58,0.8063266033079631,1.4553773075593721,stand65j14,stand65j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.4553773075593721-stand65j58),stand65j14,stand65j58,0.948427838239876,1.2447369771100414,stand65j14,stand65j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            # self.joint_position_state=[stand65j14+(0.948427838239876-stand65j14),stand65j58-(1.2447369771100414-stand65j58),stand65j14,stand65j58,0.948427838239876,1.2447369771100414,stand65j14,stand65j58]
            # self.joints_states.position = self.joint_position_state
            # self.pub.publish(self.joints_states)
            # rospy.sleep(walkingtime)
            self.joint_position_state=[stand65j14,stand65j58,stand65j14-(stand65j14-0.6569451124922087),stand65j58+(1.4051585874080312-stand65j58),stand65j14,stand65j58,0.6569451124922087,1.4051585874080312]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[stand65j14,stand65j58,stand65j14-(stand65j14-0.6810971765413939),stand65j58+(1.4051585874080312-stand65j58),stand65j14,stand65j58,0.6810971765413939,1.5199932261951814]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[stand65j14,stand65j58,stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.5199932261951814-stand65j58),stand65j14,stand65j58,0.8063266033079631,1.4553773075593721]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[stand65j14,stand65j58,stand65j14-(0.8063266033079631-stand65j14)*2,stand65j58-(1.4553773075593721-stand65j58),stand65j14,stand65j58,0.948427838239876,1.2447369771100414]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            self.joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(walkingtime)
            print('right')

        elif key_msg.button_cross == 1:   
            # rotation
            self.joint_position_state=[stand50j14,stand50j58,stand50j14,stand50j58,stand50j14,stand50j58,stand50j14,stand50j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            # right leg front
            self.joint_position_state=[stand50j14+angle,stand50j58-angle,stand50j14+angle,stand50j58-angle,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-0.785398,2.6179,stand50j14+angle,stand50j58-angle,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-0.785398,0,stand50j14+angle,stand50j58-angle,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-0.785398+g30*4/3,stand50j58-angle-g30*2,stand50j14+angle,stand50j58-angle,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            # #pierna frente izquierda
            self.joint_position_state=[-0.785398+g30*4/3,stand50j58-angle-g30*2,-0.785398,2.6179,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-0.785398+g30*4/3,stand50j58-angle-g30*2,-0.785398,0,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-0.785398+g30*4/3,stand50j58-angle-g30*2,-0.785398+g30*4/3,stand50j58-angle-g30*2,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)


            self.joint_position_state=[-0.785398+g30*4/3,stand50j58-angle-g30*2,-0.785398+g30*4/3,stand50j58-angle-g30*2,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-0.785398+g30*4/3-angle2,stand50j58-angle-g30*2+angle2,-0.785398+g30*4/3-angle2,stand50j58-angle-g30*2+angle2,stand50j14-angle,stand50j58-angle,stand50j14-angle,stand50j58-angle]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-0.785398+g30*4/3-angle2, stand50j58-angle-g30*2+angle2*2, -0.785398+g30*4/3-angle2, stand50j58-angle-g30*2+angle2*2, stand50j14-angle,stand50j58-angle-angle2,stand50j14-angle,stand50j58-angle-angle2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-0.785398+g30*4/3-angle2, stand50j58-angle-(g30/2)*2+angle2*2, -0.785398+g30*4/3-angle2, stand50j58-angle-(g30/2)*2+angle2*2, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)


            #right leg front top stair (third)
            self.joint_position_state=[-g30*7/3,g30*5, -g45+g30*4/3-angle2, stand50j58-angle-(g30/2)*2+angle2*2, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*7/3,g30*3, -g45+g30*4/3-angle2, stand50j58-angle-(g30/2)*2+angle2*2, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g45,g30*3, -g45+g30*4/3-angle2, stand50j58-angle-(g30/2)*2+angle2*2, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
        
            #left leg front top stair (third)
            self.joint_position_state=[-g45,g30*3,-g30*7/3,g30*5, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g45,g30*3,-g30*7/3,g30*3, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g45,g30*3,-g45,g30*3, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58-angle-angle2-g30]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            

            # right leg back floor 
            self.joint_position_state=[-g45,g30*3,-g45,g30*3, stand50j14-angle,stand50j58-angle-angle2-g30,-g30/2,stand50j58-angle-angle2-g30]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g45,g30*3,-g45,g30*3, stand50j14-angle,stand50j58-angle-angle2-g30,-g30/2,stand50j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g45,g30*3,-g45,g30*3, stand50j14-angle,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            # left leg back floor
            self.joint_position_state=[-g45,g30*3,-g45,g30*3, -g30/2,stand50j58-angle-angle2-g30,stand50j14-angle,stand50j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g45,g30*3,-g45,g30*3, -g30/2,stand50j58,stand50j14-angle,stand50j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g45,g30*3,-g45,g30*3, stand50j14-angle,stand50j58,stand50j14-angle,stand50j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            # Body motion 
            self.joint_position_state=[-g45,g30*4,-g45,g30*4, stand50j14,stand50j58-g30,stand50j14,stand50j58-g30]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g45,g30*5,-g45,g30*5, stand50j14,stand50j58-g30*2,stand50j14,stand50j58-g30*2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            # Body motion 
            # self.joint_position_state=[-g45,g30*5,-g45,g30*5, stand50j14,stand50j58-g30*2,stand50j14-angle,stand50j58-g45]
            # self.joints_states.position = self.joint_position_state
            # self.pub.publish(self.joints_states)
            # rospy.sleep(time)


            # right leg back (first step)
            self.joint_position_state=[-g45,g30*5,-g45,g30*5, stand50j14,stand50j58-g30*2,-g30,stand50j58-g30*2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g45,g30*5,-g45,g30*5, stand50j14,stand50j58-g30*2,-g30,stand50j58+g45/2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g45,g30*5,-g45,g30*5, stand50j14,stand50j58-g30*2, 0,stand50j58+g30/2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            # left leg back (first step)
            self.joint_position_state=[-g45,g30*5,-g45,g30*5, -g30,stand50j58-g30*2, 0,stand50j58+g30/2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g45,g30*5,-g45,g30*5, -g30,stand50j58+g45/2, 0,stand50j58+g30/2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g45,g30*5,-g45,g30*5, 0,stand50j58+g30/2, 0,stand50j58+g30/2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            # Body motion 
            self.joint_position_state=[-g30,g30*5,-g30,g30*5, g30,stand50j58-g30*2/3, g30,stand50j58-g30*2/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            # frontal derecha
            self.joint_position_state=[-0.785398,g30*5,-g30,g30*5, g30,stand50j58-g30*2/3, g30,stand50j58-g30*2/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*2,g30*4,-g30,g30*5, g30,stand50j58-g30*2/3, g30,stand50j58-g30*2/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*2,g30*3,-g30,g30*5, g30,stand50j58-g30*2/3, g30,stand50j58-g30*2/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30,g30*3,-g30,g30*5, g30,stand50j58-g30*2/3, g30,stand50j58-g30*2/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            # frontal izquierda
            self.joint_position_state=[-g30,g30*3,-0.785398,g30*5, g30,stand50j58-g30*2/3, g30,stand50j58-g30*2/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30,g30*3,-g30*2,g30*4, g30,stand50j58-g30*2/3, g30,stand50j58-g30*2/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30,g30*3,-g30*2,g30*3, g30,stand50j58-g30*2/3, g30,stand50j58-g30*2/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30,g30*3,-g30,g30*3, g30,stand50j58-g30*2/3, g30,stand50j58-g30*2/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            # # checkear el movimiento del cuerpo
            
            # Body motion 
            self.joint_position_state=[-g30*2/3,g30*4,-g30*2/3,g30*4, stand50j14-angle,g30*5/3,stand50j14-angle,g30*5/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*4/3,g30*5,-g30*4/3,g30*5, stand50j14+g30*2/3,g30,stand50j14+g30*2/3,g30]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*6/3,g30*5,-g30*6/3,g30*5, stand50j14+g30*4/3,g30,stand50j14+g30*4/3,g30]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            #right leg back top stair (third)
            self.joint_position_state=[-g30*6/3,g30*5,-g30*6/3,g30*5, stand50j14+g30*4/3,g30,-g30*7/3,0]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*6/3,g30*5,-g30*6/3,g30*5, stand50j14+g30*4/3,g30,-g30*7/3,g30*3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*6/3,g30*5,-g30*6/3,g30*5, stand50j14+g30*4/3,g30,-g30*7/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*6/3,g30*5,-g30*6/3,g30*5, stand50j14+g30*4/3,g30,g30,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*6/3,g30*5,-g30*6/3,g30*5, stand50j14+g30*4/3,g30,g30*4/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*6/3,g30*5,-g30*6/3,g30*5, stand50j14+g30*4/3,g30,g30*4/3,g30*5-g30*1/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            #left leg back top stair (third)
            self.joint_position_state=[-g30*6/3,g30*5,-g30*6/3,g30*5, -g30*7/3,0,g30*4/3,g30*5-g30*1/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*6/3,g30*5,-g30*6/3,g30*5, -g30*7/3,g30*3,g30*4/3,g30*5-g30*1/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*6/3,g30*5,-g30*6/3,g30*5, -g30*7/3,g30*5,g30*4/3,g30*5-g30*1/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*6/3,g30*5,-g30*6/3,g30*5, g30,g30*5,g30*4/3,g30*5-g30*1/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*6/3,g30*5,-g30*6/3,g30*5, g30*4/3,g30*5,g30*4/3,g30*5-g30*1/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*6/3,g30*5,-g30*6/3,g30*5, g30*4/3,g30*5-g30*1/3,g30*4/3,g30*5-g30*1/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)


            # Body motion 
            self.joint_position_state=[-g30*4/3,g30*5,-g30*4/3,g30*5, g30*4/3,g30*5-g30*1/3,g30*4/3,g30*5-g30*1/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*3/3,g30*5,-g30*3/3,g30*5, g30*2/3,g30*5,g30*2/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g30*2/3,g30*5,-g30*2/3,g30*5, -g30*2/3,g30*5,-g30*2/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
        
        elif key_msg.button_square == 1:
            #body motion
            self.joint_position_state=[g30*2/3,g30*5,g30*2/3,g30*5, -g30*3/3,g30*5,-g30*3/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            #frontal derecha
            self.joint_position_state=[-g45,g30*5,g30*2/3,g30*5, -g30*3/3,g30*5,-g30*3/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[-g45,g30,g30*2/3,g30*5, -g30*3/3,g30*5,-g30*3/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14+g30*2/3,g30*1/3,g30*2/3,g30*5, -g30*3/3,g30*5,-g30*3/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            #frontal izquierda
            self.joint_position_state=[stand50j14+g30*2/3,g30*1/3,-g45,g30*5, -g30*3/3,g30*5,-g30*3/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14+g30*2/3,g30*1/3,-g45,g30, -g30*3/3,g30*5,-g30*3/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14+g30*2/3,g30*1/3,stand50j14+g30*2/3,g30*1/3, -g30*3/3,g30*5,-g30*3/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)


            #body motion
            self.joint_position_state=[stand50j14-angle,g30*5/3,stand50j14-angle,g30*5/3, -g30*2/3,g30*4,-g30*2/3,g30*4]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14-g30*1/3,g30*2,stand50j14-g30*1/3,g30*2, -g30*3/3,g30*4,-g30*3/3,g30*4]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14-g30*1/3,g30*3,stand50j14-g30*1/3,g30*3, -g30*3/3,g30*3,-g30*3/3,g30*3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14-g30*1/3,g30*4,stand50j14-g30*1/3,g30*4, -g30*3/3,g30*2,-g30*3/3,g30*2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            #frontal derecha
            self.joint_position_state=[0,g30*4,stand50j14-g30*1/3,g30*4, -g30*3/3,g30*2,-g30*3/3,g30*2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[0,g30,stand50j14-g30*1/3,g30*4, -g30*3/3,g30*2,-g30*3/3,g30*2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[0,stand50j58-g30,stand50j14-g30*1/3,g30*4, -g30*3/3,g30*2,-g30*3/3,g30*2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58-g30,stand50j14-g30*1/3,g30*4, -g30*3/3,g30*2,-g30*3/3,g30*2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            # frontal izquierda
            self.joint_position_state=[stand50j14,stand50j58-g30,0,g30*4, -g30*3/3,g30*2,-g30*3/3,g30*2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58-g30,0,g30, -g30*3/3,g30*2,-g30*3/3,g30*2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58-g30,0,stand50j58-g30, -g30*3/3,g30*2,-g30*3/3,g30*2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58-g30,stand50j14,stand50j58-g30, -g30*3/3,g30*2,-g30*3/3,g30*2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            #trasera derecha
            self.joint_position_state=[stand50j14,stand50j58-g30,stand50j14,stand50j58-g30, -g30*3/3,g30*2,-g30*7/3,g30*2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58-g30,stand50j14,stand50j58-g30, -g30*3/3,g30*2,-g30*7/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            #trasera izquierda
            self.joint_position_state=[stand50j14,stand50j58-g30,stand50j14,stand50j58-g30, -g30*7/3,g30*2,-g30*7/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58-g30,stand50j14,stand50j58-g30, -g30*7/3,g30*5,-g30*7/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            # body 
            self.joint_position_state=[stand50j14,stand50j58,stand50j14,stand50j58, -g30*7/3,g30*4,-g30*7/3,g30*4]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            #back derecha
            self.joint_position_state=[stand50j14,stand50j58,stand50j14,stand50j58, -g30*7/3,g30*4,-g30*3,g30*4]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58,stand50j14,stand50j58, -g30*7/3,g30*4,-g30*3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58,stand50j14,stand50j58, -g30*7/3,g30*4,-g30*4/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            #back izquierda
            self.joint_position_state=[stand50j14,stand50j58,stand50j14,stand50j58, -g30*3,g30*4,-g30*4/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58,stand50j14,stand50j58, -g30*3,g30*5,-g30*4/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58,stand50j14,stand50j58, -g30*4/3,g30*5,-g30*4/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            #frente derecha 
            self.joint_position_state=[0,stand50j58,stand50j14,stand50j58, -g30*4/3,g30*5,-g30*4/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[0,stand50j58-g30,stand50j14,stand50j58, -g30*4/3,g30*5,-g30*4/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[0,stand50j58-g30*2,stand50j14,stand50j58, -g30*4/3,g30*5,-g30*4/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58-g30*2,stand50j14,stand50j58, -g30*4/3,g30*5,-g30*4/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            #frente izquierda
            self.joint_position_state=[stand50j14,stand50j58-g30*2,0,stand50j58, -g30*4/3,g30*5,-g30*4/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58-g30*2,0,stand50j58-g30, -g30*4/3,g30*5,-g30*4/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58-g30*2,0,stand50j58-g30*2, -g30*4/3,g30*5,-g30*4/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14,stand50j58-g30*2,stand50j14,stand50j58-g30*2, -g30*4/3,g30*5,-g30*4/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            # # body
            self.joint_position_state=[stand50j14,stand50j58-g30,stand50j14,stand50j58-g30, -g30*4/3,g30*4,-g30*4/3,g30*4]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14+g30*1/3,stand50j58-g30,stand50j14+g30*1/3,stand50j58-g30, -g30*3/3,g30*3,-g30*3/3,g30*3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14+g30*2/3,stand50j58-g30*5/3,stand50j14+g30*2/3,stand50j58-g30*5/3, -g30*2/3,g30*3,-g30*2/3,g30*3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14+g30*3/3,stand50j58-g30*5/3,stand50j14+g30*3/3,stand50j58-g30*5/3, 0,g30*2,0,g30*2]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            #back derecha
            self.joint_position_state=[stand50j14+g30*3/3,stand50j58-g30*5/3,stand50j14+g30*3/3,stand50j58-g30*5/3, 0,g30*2,-g30*5/3,g30*3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14+g30*3/3,stand50j58-g30*5/3,stand50j14+g30*3/3,stand50j58-g30*5/3, 0,g30*2,-g30*5/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14+g30*3/3,stand50j58-g30*5/3,stand50j14+g30*3/3,stand50j58-g30*5/3, 0,g30*2,stand50j14+g30*2/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14+g30*3/3,stand50j58-g30*5/3,stand50j14+g30*3/3,stand50j58-g30*5/3, 0,g30*2,stand50j14+g30*3/3,stand50j58-g30*5/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)

            self.joint_position_state=[stand50j14+g30*3/3,stand50j58-g30*5/3,stand50j14+g30*3/3,stand50j58-g30*5/3, -g30*5/3,g30*3,stand50j14+g30*3/3,stand50j58-g30*5/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14+g30*3/3,stand50j58-g30*5/3,stand50j14+g30*3/3,stand50j58-g30*5/3, -g30*5/3,g30*5,stand50j14+g30*3/3,stand50j58-g30*5/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14+g30*3/3,stand50j58-g30*5/3,stand50j14+g30*3/3,stand50j58-g30*5/3, stand50j14+g30*2/3,g30*5,stand50j14+g30*3/3,stand50j58-g30*5/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)
            self.joint_position_state=[stand50j14+g30*3/3,stand50j58-g30*5/3,stand50j14+g30*3/3,stand50j58-g30*5/3,stand50j14+g30*3/3,stand50j58-g30*5/3,stand50j14+g30*3/3,stand50j58-g30*5/3]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time)





        elif key_msg.button_triangle == 1:
            self.joint_position_state=[stand50j14,stand50j58,stand50j14,stand50j58,stand50j14,stand50j58,stand50j14,stand50j58]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)   
        
        elif key_msg.button_circle == 1:
            self.joint_position_state=[0,0,0,0,0,0,0,0]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)  
            rospy.sleep(time) 
            self.joint_position_state=[-g30*2/3,g30*5,-g30*2/3,g30*5, -g30*2/3,g30*5,-g30*2/3,g30*5]
            self.joints_states.position = self.joint_position_state
            self.pub.publish(self.joints_states)
            rospy.sleep(time) 


        elif key_msg.button_ps == 1:   #HOME_POS
            rospy.is_shutdown()

        # self.pub.publish(self.joints_states)






    def __init__(self):

        rospy.init_node('joystick')
        self.r = rospy.Rate(10)
        self.pub = rospy.Publisher('joint_goals', JointState, queue_size=1)
        rospy.Subscriber('/status', Status, self.key_cb,queue_size=1)

        




    def loop(self):
        self.r.sleep()

if __name__ == '__main__':
    key2vel = Key2Vel()
    while not rospy.is_shutdown():
        key2vel.loop()