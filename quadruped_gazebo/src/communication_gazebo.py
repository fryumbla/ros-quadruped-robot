#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState

from std_msgs.msg import Header
from std_msgs.msg import Float64

# global joint_goals


joint_goals = JointState()
joint_goals.position = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

def callback(data):
    joint_goals.header = Header()
    joint_goals.header.stamp = rospy.Time.now()
    joint_goals.name = data.name
    joint_goals.position = data.position
    

def main():

    rospy.init_node("communication_gazebo")

    pub1 = rospy.Publisher('quadruped/joint1_position_controller/command', Float64, queue_size=1)
    pub2 = rospy.Publisher('quadruped/joint2_position_controller/command', Float64, queue_size=1)
    pub3 = rospy.Publisher('quadruped/joint3_position_controller/command', Float64, queue_size=1)
    pub4 = rospy.Publisher('quadruped/joint4_position_controller/command', Float64, queue_size=1)
    pub5 = rospy.Publisher('quadruped/joint5_position_controller/command', Float64, queue_size=1)
    pub6 = rospy.Publisher('quadruped/joint6_position_controller/command', Float64, queue_size=1)
    pub7 = rospy.Publisher('quadruped/joint7_position_controller/command', Float64, queue_size=1)
    pub8 = rospy.Publisher('quadruped/joint8_position_controller/command', Float64, queue_size=1)

    # rospy.Subscriber('joint_goals', JointState, callback)
    
    while not rospy.is_shutdown():
        rospy.Subscriber('/joint_goals', JointState, callback, queue_size=1)
        pub1.publish(joint_goals.position[0])
        pub2.publish(joint_goals.position[1])
        pub3.publish(joint_goals.position[2])
        pub4.publish(joint_goals.position[3])
        pub5.publish(joint_goals.position[4])
        pub6.publish(joint_goals.position[5])
        pub7.publish(joint_goals.position[6])
        pub8.publish(joint_goals.position[7])
        rate = rospy.Rate(10)
        

if __name__ == '__main__':
    main()