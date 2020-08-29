#!/usr/bin/env python
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def main():
    rospy.init_node("stand_up")
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    joints_states = JointState()

    stand50j14=0.41944732836554044
    stand50j58=1.719784407902978

    stand65j14=0.7592545338404827
    stand65j58=1.169485889801056

    while not rospy.is_shutdown():

        joints_states.header = Header()
        joints_states.header.stamp = rospy.Time.now()
        joints_states.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1','front_left_joint2', 'back_left_joint1', 'back_left_joint2', 'back_right_joint1','back_right_joint2']
        
        time=1
        rate = rospy.Rate(10) # 10hz  
             
        # joint_position_state=[-1,2,-1,2,-1,2,-1,2]
        # joint_position_state=[0,0,0,0,0,0,0,0]
        number = input ("Enter number: ")
        if (number==1):
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
            joint_position_state=[0.4194,1.7198,stand65j14,stand65j58,0.4194,1.7198,stand65j14,stand65j58]	
            joints_states.position = joint_position_state
            pub.publish(joints_states)	
            rospy.sleep(time)
            joint_position_state=[0.7047391,1.009487,stand65j14,stand65j58,0.9484119,1.244768,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[stand65j14,stand65j58,0.4194,1.7198,stand65j14,stand65j58,0.4194,1.7198]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[stand65j14,stand65j58,0.7047391,1.009487,stand65j14,stand65j58,0.9484119,1.244768]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
        if (number==7):
            # backward
            joint_position_state=[0.4194,1.7198,stand65j14,stand65j58,0.4194,1.7198,stand65j14,stand65j58]	
            joints_states.position = joint_position_state
            pub.publish(joints_states)	
            rospy.sleep(time)
            joint_position_state=[stand65j14+0.0542609,stand65j58+0.159883,stand65j14,stand65j58,stand65j14-0.1894119,stand65j58-0.075398,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[stand65j14,stand65j58,0.4194,1.7198,stand65j14,stand65j58,0.4194,1.7198]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[stand65j14,stand65j58,stand65j14+0.0542609,stand65j58+0.159883,stand65j14,stand65j58,stand65j14-0.1894119,stand65j58-0.075398]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)



        if (number==8):
            # segundo paso
            joint_position_state=[0.4194,1.7198,stand65j14,stand65j58,0.4194,1.7198,stand65j14,stand65j58]	
            joints_states.position = joint_position_state
            pub.publish(joints_states)	
            rospy.sleep(time)
            joint_position_state=[0.4194,1.7198,stand65j14,stand65j58,0.4194,1.7198,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[stand65j14,stand65j58,0.4194,1.7198,stand65j14,stand65j58,0.4194,1.7198]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[stand65j14,stand65j58,0.7047391,1.009487,stand65j14,stand65j58,0.9484119,1.244768]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
        if (number==9):
            # segundo paso
            joint_position_state=[0.4194,1.7198,stand65j14,stand65j58,0.4194,1.7198,stand65j14,stand65j58]	
            joints_states.position = joint_position_state
            pub.publish(joints_states)	
            rospy.sleep(time)
            joint_position_state=[-0.785398,2.6179,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[-0.785398,0,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[0,0,stand65j14,stand65j58,stand65j14,stand65j58,stand65j14,stand65j58]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[0,0,stand65j14+0.1745329,stand65j58+0.1745329,stand65j14-0.1745329,stand65j58-0.1745329,stand65j14-0.1745329,stand65j58-0.1745329]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            joint_position_state=[0,0,stand65j14+0.1745329,stand65j58+0.1745329,stand65j14-0.1745329,stand65j58-0.1745329,stand65j14-0.1745329,stand65j58-0.1745329]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            # rospy.sleep(time)
                    


if __name__ == '__main__':
    main()