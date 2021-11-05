#!/usr/bin/env python


import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
#from sympy import *
#from time import time
#from mpmath import radians
#import tf


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
    #L0 = 0.3
    #L = 0.4
    #def cal_IK(x,y):
    #    L1 = L2 = L
    #    q2 = acos(((x-L0/2)*(x-L0/2) + y*y + L1*L1 + L2*L2)/(2*L1*L2))
    #    r = atan((L2*sin(q2))/(L1+L2*cos(q2)))
    #    q1 = atan((x-L0/2)/y)-r
    #    ang = [q1, q2]
    #    return ang

    
    
    while not rospy.is_shutdown():

        joints_states.header = Header()
        joints_states.header.stamp = rospy.Time.now()
        joints_states.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1','front_left_joint2', 'back_left_joint1', 'back_left_joint2', 'back_right_joint1','back_right_joint2']
        time=1
        walkingtime=0.5
        rate = rospy.Rate(10) # 10hz  
             
        # joint_position_state=[-1,2,-1,2,-1,2,-1,2]
        # joint_position_state=[0,0,0,0,0,0,0,0]
        c = input ("Enter command: ")
        

        
        # start

        if (c == 1):
            if(joint_position_state == [0,0,0,0,0,0,0,0]):
                joint_position_state = [-0.827/2, 0.827,-0.827/2, 0.827, -0.827/2, 0.827, -0.827/2, 0.827]
                joints_states.position = joint_position_state
                pub.publish(joints_states)
                rospy.sleep(time)
                joint_position_state = [0.989,0.827,0.989,0.827,0.989,0.827,0.989,0.827]
                joints_states.position = joint_position_state
                pub.publish(joints_states)
                
            
        # finish
        if (c == 2):
            if(joint_position_state != [0,0,0,0,0,0,0,0]):
                
                joint_position_state = [-1.57,1.57,-1.57,1.57,-1.57,1.57,-1.57,1.57]
                joints_states.position = joint_position_state
                pub.publish(joints_states)
                rospy.sleep(time)
                joint_position_state=[0,0,0,0,0,0,0,0]
                joints_states.position = joint_position_state
                pub.publish(joints_states)
                
        if (c == 3):
            joint_position_state=[0,0,0,0,0,0,0,0]
            joints_states.position = joint_position_state
            pub.publish(joints_states)

        if (c == 4):
            joint_position_state = [-1.57,1.57,-1.57,1.57,-1.57,1.57,-1.57,1.57]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            for ang in [-1.57,0,0.01]:
                joint_position_state = [ang,-ang,ang,-ang,ang,-ang,ang,-ang]
                joints_states.position = joint_position_state
                pub.publish(joints_states)
                rospy.sleep(time)

        if(c == 5):
            joint_position_state = [-1.57,1.57,-1.57,1.57,-1.57,1.57,-1.57,1.57]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[0,0,0,0,0,0,0,0]
            joints_states.position = joint_position_state
            pub.publish(joints_states)

        if(c == 6):
            frj1 = -0.827/2
            frj2 = 0.827
            
            


if __name__ == '__main__':
    main()