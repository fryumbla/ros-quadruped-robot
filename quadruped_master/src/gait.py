#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math
from math import pi, cos, sin, acos, atan2

global current_position
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

def send_joints(list_joints):
    global current_position
    joints_states.position = list_joints
    pub.publish(joints_states)
    current_position = list_joints


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

def gait(foot_number, length, time_delay): #Front foot? True or False
    steps = 10
    length = length/2
    global current_position
    n = 2*foot_number -1
    joint_position_state = current_position
    #1rst leg gait
    first_position = FK(current_position[n-1], current_position[n],True)
    #print("first position: ",first_position)
    if length>=0:
        if foot_number>2: #Para las patas traseras
            posd = plan_circle(first_position[0]-length, first_position[1], length, 0,180, True, True, steps)
        else: #Patas delanteras
            posd = plan_circle(first_position[0]+length, first_position[1], length, 0,180, False, True, steps)
    else:
        if foot_number>2: #Para las patas traseras
            posd = plan_circle(first_position[0]-length, first_position[1], length, 0,180, True, False, steps)
        else: #Patas delanteras
            print(first_position[0]+length, first_position[1])
            posd = plan_circle(first_position[0]+length, first_position[1], length, 0,180,  False, False, steps)
            print(posd)

    for p in posd:
        #print("Position x,z: ",p)
        j = IK(p[0], p[1],True)
        #print("Joints: ",j)
        joint_position_state[n]=j[1]
        joint_position_state[n-1]=j[0]
        send_joints(joint_position_state)
        rospy.sleep(time_delay)
    pass

def dummy_traslation(x,z, time_delay):
    global current_position
    joint_position_goal = [0,0,0,0,0,0,0,0]
    pre_position = [(0,0),(0,0),(0,0),(0,0)]
    final_position = [(0,0),(0,0),(0,0),(0,0)]
    for i in range(4):
        x_new = x
        n = 2*i 
        base_new_arm = angles_new_arm(current_position[n],current_position[n+1])
        point_new_arm = FK(base_new_arm[0],base_new_arm[1],False)
        pre_position[i] = point_new_arm

        if i >= 2:
            print("back")
            x_new = -x
        else:
            print("frontal")

        new_point = (point_new_arm[0]+x_new, point_new_arm[1]+z)
        final_position[i] = new_point

    steps = 20

    for step in range(steps+1):
        interpolated_position = [(0,0),(0,0),(0,0),(0,0)]
        #print("interpolacion")

        for a in range(4):
            m = 2*a
            x_interp = pre_position[a][0] + (final_position[a][0] - pre_position[a][0]) * (step / steps)
            z_interp = pre_position[a][1] + (final_position[a][1] - pre_position[a][1]) * (step / steps)
            #print(x_interp,z_interp)

            new_angles = IK(x_interp,z_interp,False)
            angles = angles_buddy_arm(new_angles[0],new_angles[1])
            joint_position_goal[m] = angles[0]
            joint_position_goal[m+1] = angles[1]
        send_joints(joint_position_goal)
        #print(joint_position_goal)
        rospy.sleep(time_delay)

    pass

def dummy_rotation(angle):

    print("Rotation", angle)
    d = 0.22 #Distancia del dummy entra pata y pata
    angle = - np.pi*angle/180
    z = (d)*sin(angle)
    x = -(d)*(cos(angle)) + d
    #z = 0.22
    #x = 0.22
    #FRONT
    #print("Dummy rotation")
    global current_position
    first_position = FK(current_position[0], current_position[1],True)
    #print("first position: ", first_position)
    base_new_arm = angles_new_arm(current_position[0],current_position[1])
    point_new_arm = FK(base_new_arm[0],base_new_arm[1],False)
    new_point = (point_new_arm[0]-x, point_new_arm[1]-z)
    print("first position new arm: ",point_new_arm)
    print("new point: ",new_point)
    new_angles = IK(new_point[0],new_point[1],False)
    angles = angles_buddy_arm(new_angles[0],new_angles[1])
    #print("Angles new arm: ",new_angles )
    #print("Angles buddy arm: ",angles )

    #BACK
    base_new_arm_b = angles_new_arm(current_position[0],current_position[1])
    point_new_arm_b = FK(base_new_arm_b[0],base_new_arm_b[1],False)
    new_point_b = (point_new_arm_b[0]+x, point_new_arm_b[1]+z)
    print("first position new arm: ",point_new_arm_b)
    print("new point: ",new_point_b)
    new_angles_b = IK(new_point_b[0],new_point_b[1],False)
    angles_b = angles_buddy_arm(new_angles_b[0],new_angles_b[1])
    #print("Angles buddy arm: ",angles_b )

    joint_position_state=[angles[0],angles[1] ,angles[0],angles[1],angles_b[0],angles_b[1],angles_b[0],angles_b[1]] # stand up principal
    send_joints(joint_position_state)

    rospy.sleep(0.5)
    pass

def movement(speed):
    #speed variara de -5 a 5
    min_speed = 0
    max_speed = 5
    delay_min = 0.1  # en s
    delay_max = 0.5  # en s
    length = 0.15

    if speed<0:
        length = -length
 
    # Suponiendo que `velocidad` es el valor actual de la velocidad
    time_delay = delay_max - ((abs(speed) - min_speed) / (max_speed - min_speed)) * (delay_max - delay_min)
    print(time_delay)
    print(speed)
    
    
    while speed!=0:
        print("Traslation")
        dummy_traslation(-length/2,0,time_delay)
        rospy.sleep(0.05)
        print("Front Gait")
        if speed>0: #Cuando Avanza
            gait(1,length,time_delay)
            gait(2,length,time_delay)
            
            dummy_traslation(4*length/2,0, time_delay)
            rospy.sleep(1)

            gait(3,length,time_delay)
            gait(4,length,time_delay)
        
        if speed<0: #Cuando retrocede
            gait(4,length,time_delay)
            gait(3,length,time_delay)
            
            dummy_traslation(4*length/2,0, time_delay)
            rospy.sleep(1)

            gait(2,length,time_delay)
            gait(1,length,time_delay)

        dummy_traslation(-length/2,0, time_delay)

    pass

def main():
    stand50j14=0.41944732836554044
    stand50j58=1.719784407902978


    stand_1=0.3
    stand_2=1.4

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
            print("Stand")
            joint_position_state=[stand_1, stand_2,stand_1,stand_2,stand_1,stand_2,stand_1,stand_2] # stand up principal
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
            rospy.sleep(1)
            user = input ("Enter Speed: ")
            velocity = int(user)
            movement(velocity)
            pass
        if (number ==4):
            rospy.sleep(1)
            dummy_traslation(0.1,0, 0.1)



if __name__ == '__main__':
    rospy.init_node("motion_control")
    #pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    joints_states = JointState()

    main()
    # dummy_traslation(0.1,0)
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
