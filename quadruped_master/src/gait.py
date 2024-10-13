#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math
from math import pi, cos, sin, acos, atan2

current_position = [0,0,0,0,0,0,0,0]

def FK(theta1,theta2, base_on_dummy):
    #t1 = theta1*180/np.pi
    #t2 = theta2*180/np.pi
    if(base_on_dummy):
        theta1 = -theta1
        theta2 = -theta2
    l1 = 0.4
    l2 = 0.4

    x = l1*np.cos(theta1) + l2*np.cos(theta1 + theta2)
    z = l1*np.sin(theta1) + l2*np.sin(theta1 + theta2)
    return (x,z)

def IK(pos_x, pos_z, base_on_dummy):
    global current_position
    l1 = 0.4
    l2 = 0.4
    distancia = np.sqrt(pos_x**2 + pos_z**2)
    if distancia > (l1 + l2) or distancia < abs(l1 - l2):
        print("El punto objetivo está fuera del alcance del robot.")
    else:
        cosbeta = ( pos_x ** 2 + pos_z ** 2 - l1 ** 2 - l2 ** 2 ) / ( 2.0 * l1 * l2 )
        beta1 = acos( cosbeta )
        beta2 = -beta1
        A  , B = l1 + l2 * cosbeta, l2 * sin( beta1 )
        if(base_on_dummy):
            alpha1 = - atan2( pos_z * A + pos_x * B, pos_x * A - pos_z * B )
        else:
            alpha1 = atan2( pos_z * A - pos_x * B, pos_x * A + pos_z * B )
        #print("beta1:alpha1, beta2:alpha2")
        #print("(",alpha1,beta1,"),(",alpha2,beta2, ")")
    return alpha1,beta1

def angles_new_arm(theta1, theta2):
    beta = theta2
    alpha = 3.14 - (beta + theta1)
    return (alpha,beta)

def angles_buddy_arm(alpha, beta):
    theta2 = beta
    theta1 = 3.14 -(beta+alpha)
    return (theta1, theta2)

def plan_circle( center_x : float , center_y : float , r : float , theta_o : float  , theta_f : float , sentido_x : bool, sentido_y : bool, steeps):
    pos = []
    if (sentido_x and sentido_y):
        for theta in range(theta_o, theta_f + 1, steeps):
            position_z = center_y + r*math.sin(theta*math.pi/180)
            position_x = center_x + r*math.cos(theta*math.pi/180)
            pos.append((position_x,position_z))
    elif (not(sentido_x) and sentido_y):
        for theta in range(theta_o, theta_f + 1, steeps):
            position_z = center_y + r*math.sin(theta*math.pi/180)
            position_x = center_x - r*math.cos(theta*math.pi/180)
            pos.append((position_x,position_z))
    elif (sentido_x and not(sentido_y)):
        for theta in range(theta_o, theta_f + 1, steeps):
            position_z = center_y - r*math.sin(theta*math.pi/180)
            position_x = center_x + r*math.cos(theta*math.pi/180)
            pos.append((position_x,position_z))
    else:
        for theta in range(theta_o, theta_f + 1, steeps):
            position_z = center_y - r*math.sin(theta*math.pi/180)
            position_x = center_x - r*math.cos(theta*math.pi/180)
            pos.append((position_x,position_z))

    return pos

def gait(foot_number, length): #Front foot? True or False
    steps = 20
    global current_position
    n = 2*foot_number -1
    joint_position_state = current_position
    #1rst leg gait
    first_position = FK(current_position[n-1], current_position[n],True)
    print("first position: ",first_position)
    if length>=0:
        if foot_number>2: #Para las patas traseras
            posd = plan_circle(first_position[0]-length, first_position[1], length, 0,180, True, True, steps)
        else: #Patas delanteras
            posd = plan_circle(first_position[0]+length, first_position[1], length, 0,180, False, True, steps)
    else:
        if foot_number>2: #Para las patas traseras
            print("ja")
            posd = plan_circle(first_position[0]-length, first_position[1], length, 0,180, True, False, steps)
        else: #Patas delanteras
            print("hoa")
            print(first_position[0]+length, first_position[1])
            posd = plan_circle(first_position[0]+length, first_position[1], length, 0,180,  False, False, steps)
            print(posd)

    for p in posd:
        print("Position x,z: ",p)
        j = IK(p[0], p[1],True)
        print("Joints: ",j)
        joint_position_state[n]=j[1]
        joint_position_state[n-1]=j[0]
        joints_states.position = joint_position_state
        pub.publish(joints_states)
        current_position = joint_position_state
        rospy.sleep(0.8)
    pass

def dummy_traslation(x,z):
    #FRONT
    print("front displacement")
    global current_position
    first_position = FK(current_position[0], current_position[1],True)
    print("first position: ", first_position)
    base_new_arm = angles_new_arm(current_position[0],current_position[1])
    point_new_arm = FK(base_new_arm[0],base_new_arm[1],False)
    new_point = (point_new_arm[0]+x, point_new_arm[1]+z)
    print("first position new arm: ",point_new_arm)
    print("new point: ",new_point)
    new_angles = IK(new_point[0],new_point[1],False)
    angles = angles_buddy_arm(new_angles[0],new_angles[1])
    print("Angles new arm: ",new_angles )
    print("Angles buddy arm: ",angles )

    #BACK
    base_new_arm_b = angles_new_arm(current_position[0],current_position[1])
    point_new_arm_b = FK(base_new_arm_b[0],base_new_arm_b[1],False)
    new_point_b = (point_new_arm_b[0]+x, point_new_arm_b[1]+z)
    new_angles_b = IK(new_point_b[0],new_point_b[1],False)
    angles_b = angles_buddy_arm(new_angles_b[0],new_angles_b[1])

    joint_position_state=[angles[0],angles[1] ,angles[0],angles[1],angles_b[0],angles_b[1],angles_b[0],angles_b[1]] # stand up principal
    joints_states.position = joint_position_state
    pub.publish(joints_states)
    current_position = joint_position_state
    rospy.sleep(0.5)
    pass

def main():
    stand50j14=0.41944732836554044
    stand50j58=1.719784407902978


    stand_1=0.41944732836554044
    stand_2=1.2

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
            joint_position_state=[stand_1, stand_2,stand_1,stand_2,stand_1,stand_2,stand_1,stand_2] # stand up principal
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            current_position = joint_position_state

        if (number ==3):
            print("Front Gait")
            #1rst leg gait
            gait(1,0.1)
            gait(2,0.1)
            
            dummy_traslation(0.1,0)

            gait(3,0.1)
            gait(4,0.1)

            dummy_traslation(0.1,0)
            rospy.sleep(2)

            print("Retrocediendo")
            gait(1,-0.1)
            gait(2,-0.1)

            dummy_traslation(-0.1,0)
            
            gait(3,-0.1)
            gait(4,-0.1)

            dummy_traslation(-0.1,0)

            rospy.sleep(2)

            dummy_traslation(0,+0.1)
            rospy.sleep(1)

            dummy_traslation(0,-0.2)

            rospy.sleep(1)

            dummy_traslation(0,+0.2)
            pass

        if (number ==4):
            #FRONT
            print("front displacement")
            current_position = joint_position_state
            first_position = FK(current_position[0], current_position[1],True)
            print("first position: ", first_position)
            base_new_arm = angles_new_arm(current_position[0],current_position[1])
            point_new_arm = FK(base_new_arm[0],base_new_arm[1],False)
            new_point = (point_new_arm[0], point_new_arm[1]-0.077)
            print("first position new arm: ",point_new_arm)
            print("new point: ",new_point)
            new_angles = IK(new_point[0],new_point[1],False)
            angles = angles_buddy_arm(new_angles[0],new_angles[1])
            print("Angles new arm: ",new_angles )
            print("Angles buddy arm: ",angles )

            #BACK
            first_position_b = FK(current_position[4], current_position[5],True)
            base_new_arm_b = angles_new_arm(current_position[0],current_position[1])
            point_new_arm_b = FK(base_new_arm_b[0],base_new_arm_b[1],False)
            new_point_b = (point_new_arm_b[0], point_new_arm_b[1]+0.077)
            new_angles_b = IK(new_point_b[0],new_point_b[1],False)
            angles_b = angles_buddy_arm(new_angles_b[0],new_angles_b[1])

            joint_position_state=[angles[0],angles[1] ,angles[0],angles[1],angles_b[0],angles_b[1],angles_b[0],angles_b[1]] # stand up principal
            joints_states.position = joint_position_state
            pub.publish(joints_states)
            current_position = joint_position_state
            rospy.sleep(0.5)
            pass



if __name__ == '__main__':
    rospy.init_node("motion_control")
    #pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    pub = rospy.Publisher('joint_goals', JointState, queue_size=1)
    joints_states = JointState()

    main()
    # angles = (1.78,1)
    # a = FK(angles[0],angles[1],True)
    # print("Forward kinematics: ",a)
    # print("IK",IK(a[0],a[1],True))
    # b =  angles_new_arm(angles[0],angles[1])
    # print("New arm angles: ", b)
    # c = FK(b[0],b[1], False)
    # print("New arm FK:", c)
    # print("New arm IK: ",IK(c[0],c[1],False))
    #print(IK(a[0],a[1]))
    #gait(1)
