#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def main():


    rospy.init_node("stand_up")

    while not rospy.is_shutdown():
        
		# joint_position_state=[-1,2,-1,2,-1,2,-1,2]
		# joint_position_state=[0,0,0,0,0,0,0,0]
  		number = input ("Enter number: ")
		if (number==1):
			joint_position_state=[0,0,0,0,0,0,0,0]
		if (number==2):
			print("dos")
			joint_position_state=[-1,2.2,-1,2.2,-1,2.2,-1,2.2]
		if (number==3):
			joint_position_state=[0,1.57,0,1.57,0,1.57,0,1.57]
		if (number==4):
			joint_position_state=[0.5,1,0.5,1,0.5,1,0.5,1]		
		if (number==5):
			joint_position_state=[-1,0,-1,0,-1,0,-1,0]	
					



		pub = rospy.Publisher('joint_states', JointState, queue_size=10)
		# rate = rospy.Rate(1000000) # 10hz
		joints_states = JointState()
		joints_states.header = Header()
		joints_states.header.stamp = rospy.Time.now()
		joints_states.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1','front_left_joint2', 'back_left_joint1', 'back_left_joint2', 'back_right_joint1','back_right_joint2']
		joints_states.position = joint_position_state
		pub.publish(joints_states)
		rate = rospy.Rate(10) # 10hz   
		# rospy.sleep(5)
		# joints_states.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1','front_left_joint2', 'back_left_joint1', 'back_left_joint2', 'back_right_joint1','back_right_joint2']
		# joints_states.position = [0,2,0,2,0,2,0,2]
		# pub.publish(JointState)


if __name__ == '__main__':
	main()