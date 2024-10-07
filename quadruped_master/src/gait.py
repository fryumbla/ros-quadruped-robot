#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math

current_position = [0,0,0,0,0,0,0,0]

def FK(theta1,theta2):
    #t1 = theta1*180/np.pi
    #t2 = theta2*180/np.pi
    theta1 = -theta1
    theta2 = -theta2
    l1 = 0.4
    l2 = 0.4

    x = l1*np.cos(theta1) + l2*np.cos(theta1 + theta2)
    z = l1*np.sin(theta1) + l2*np.sin(theta1 + theta2)
    return (x,z)

def IK(pos_x, pos_z):
    global current_position
    l1 = 0.4
    l2 = 0.4
    distancia = np.sqrt(pos_x**2 + pos_z**2)
    if distancia > (l1 + l2) or distancia < abs(l1 - l2):
        print("El punto objetivo está fuera del alcance del robot.")
        # Aquí puedes decidir qué hacer: saltar el punto, ajustar la posición, etc.
        theta1 = current_position[0]
        theta2 = current_position[1]

    else:
        theta2 = np.arccos(np.clip(( (pos_x**2) + (pos_z**2) - (l1**2) - (l2**2) ) / (2*l1*l2), -1, 1 ))
        theta1 = -(np.arctan(pos_z/pos_x) - np.arctan((l2*np.sin(-theta2))/(l1+l2*np.cos(-theta2))))
    return (theta1,theta2)


def plan_circle( center_x : float , center_y : float , r : float , theta_o : float  , theta_f : float , sentido_x : bool, sentido_y : bool):
    pos = []
    if (sentido_x and sentido_y):
        for theta in range(theta_o, theta_f + 1, 2):
            position_z = center_y + r*math.sin(theta*math.pi/180)
            position_x = center_x + r*math.cos(theta*math.pi/180)
            pos.append((position_x,position_z))
    elif (not(sentido_x) and sentido_y):
        for theta in range(theta_o, theta_f + 1, 2):
            position_z = center_y + r*math.sin(theta*math.pi/180)
            position_x = center_x - r*math.cos(theta*math.pi/180)
            pos.append((position_x,position_z))
    elif (sentido_x and not(sentido_y)):
        for theta in range(theta_o, theta_f + 1, 2):
            position_z = center_y - r*math.sin(theta*math.pi/180)
            position_x = center_x + r*math.cos(theta*math.pi/180)
            pos.append((position_x,position_z))
    else:
        for theta in range(theta_o, theta_f + 1, 2):
            position_z = center_y - r*math.sin(theta*math.pi/180)
            position_x = center_x - r*math.cos(theta*math.pi/180)
            pos.append((position_x,position_z))

    return pos

def gait(length):
    posd = plan_circle(0.2, 0.2, 0.2, 0,180, False, True)
    print(posd)
    pass

def main():
    rospy.init_node("motion_control")
    pub = rospy.Publisher('joint_states', JointState, queue_size=1)
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

        global current_position

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
            print("Abajo")
            joint_position_state=[0,0,0,0,0,0,0,0]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            current_position = joint_position_state

        if (number==2):
            # up
            print("Up position")
            joint_position_state=[-1,2.2,-1,2.2,-1,2.2,-1,2.2]
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            rospy.sleep(time)
            joint_position_state=[stand50j14, stand50j58,stand50j14,stand50j58,stand50j14,stand50j58,stand50j14,stand50j58] # stand up principal
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            current_position = joint_position_state

        if (number ==3):
            r =0.1
            print("Front Gait")
            current_position = joint_position_state
            #1rst leg gait
            first_position = FK(current_position[0], current_position[1])
            print(first_position)
            posd = plan_circle(first_position[0]+r, first_position[1], r, 0,180, False, True)
            print(posd)
            for p in posd:
                j = IK(p[0], p[1])
                print(j)
                joint_position_state=[j[0], j[1],stand50j14,stand50j58,stand50j14,stand50j58,stand50j14,stand50j58] # stand up principal
                joints_states.position = joint_position_state
                pub.publish(joints_states)
                current_position = joint_position_state
                rospy.sleep(0.01)








if __name__ == '__main__':
    main()
    #print(FK(1.57 , 0))
    #print(IK(0.0006370613685866107, -0.7999997463454678))
    #gait(1)