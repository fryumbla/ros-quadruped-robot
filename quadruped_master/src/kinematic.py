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
from tf.transformations import quaternion_from_euler, euler_from_quaternion


rospy.init_node("kinematic_control_node")
rate = rospy.Rate(10) # 10hz  
pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

joints_states = JointState()
joints_states.header = Header()
joints_states.header.stamp = rospy.Time.now()
joints_states.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1','front_left_joint2', 'back_left_joint1', 'back_left_joint2', 'back_right_joint1','back_right_joint2']

L = L1 = L2 = float(0.4) # largo de cada eslabon de una pierna
center_to_side = 0.11    # distancia horizontal que hay entre el centro del torso hasta el primer joint de la pata


# Definir ángulos de Euler (roll, pitch, yaw) en radianes
roll = 0  # Rotación alrededor del eje X
pitch = 0 # Rotación alrededor del eje Y
yaw = 0   # Rotación alrededor del eje Z

# Convertir a cuaternión
quaternion = quaternion_from_euler(roll, pitch, yaw)

wpose = Pose()
wpose.position.z = 0.6
# Asignar al objeto wpose
wpose.orientation.x = quaternion[0]
wpose.orientation.y = quaternion[1]
wpose.orientation.z = quaternion[2]
wpose.orientation.w = quaternion[3]

arm_l = Pose() # we make the math for a 2R articulated arm from the floor for each leg 
arm_r = Pose() # we make the math for a 2R articulated arm from the floor for each leg


def start():
    """Set the quadruped to the home position."""
    joint_position_state=[-1,2.2,-1,2.2,-1,2.2,-1,2.2]
    joints_states.position = joint_position_state
    pub.publish(joints_states)
    rospy.sleep(1.00)
    joint_position_state=[0.7592545338404827,1.169485889801056,0.7592545338404827,1.169485889801056,0.7592545338404827,1.169485889801056,0.7592545338404827,1.169485889801056] # stand up principal
    joints_states.position = joint_position_state
    pub.publish(joints_states)

def set_orientation(wpose: Pose, pitch: float):
    """Allow us to change the pitch in R P Y, the rotation about "y" axis"""
    pitch = math.radians(pitch)
    quaternion = quaternion_from_euler(0, pitch, 0)
    wpose.orientation.x = quaternion[0]
    wpose.orientation.y = quaternion[1]
    wpose.orientation.z = quaternion[2]
    wpose.orientation.w = quaternion[3]
    return wpose

def calculateIK_floor(wpose: Pose):
    z = wpose.position.z

    # beta  =-math.cos(((x**2) + (z**2) - 2*(L**2))/(2*(L**2))) # angulo del link1 con respecto al suelo
    # alpha = math.atan2(z,x) + math.atan2((math.sin(beta)), (1+math.cos(beta)))# angulo del link2 con respecto al link1
    # q1 = -(alpha + beta)
    # q2 = -beta

    # alpha = math.atan2(-z, x) + math.cos( sqrt( (x**2) + (z**2) )/( 2*L ))
    # beta  = -(math.cos(( (2*L**2) - ( (x**2) + (z**2) ) )/(2*L**2 )) - pi)
    # q1 = (alpha + beta)
    # q2 = beta

    # alpha = math.radians(80)
    # x = z / math.tan(alpha)
    # beta = math.math.atan2(z, x) - alpha
    # q1 = (alpha + beta)
    # q2 = -beta
    # print(x)

    if z < 0.7:
        alpha = math.radians(85)
        beta = math.sin(z / (2 * L)) - alpha
        x = L * (math.cos(alpha + beta))
        q1 = (alpha + beta)
        q2 = -beta

        arm_l.position.x = x
        arm_r.position.x = x

        if(complex(alpha).imag != 0):
            print("alpha Impossible Angle")
        elif(complex(beta).imag != 0):
            print("beta Impossible Angle")
        elif(complex(q1).imag != 0):
            print("q1 Impossible Angle")
        elif(complex(q2).imag != 0):
            print("q2 Impossible Angle")
        else:
            joint_position_state=[q1,q2,q1,q2,q1,q2,q1,q2] # stand up principal
            joints_states.position = joint_position_state
            pub.publish(joints_states)

    else:
        wpose.position.z = 0.65
        rospy.logwarn(wpose.position.z)


    # rospy.logwarn("Beta: {}, Alpha: {}, q1: {}, q2: {}, (X Z): {}  {}\n".format(
    #     str(round(math.degrees(beta),  2)), 
    #     str(round(math.degrees(alpha), 2)), 
    #     str(round(math.degrees(q1), 2)), 
    #     str(round(math.degrees(q2), 2)), 
    #     str(round(0.4 * (math.cos(q1) + math.cos(q1 + q2)), 2)), 
    #     str(round(0.4 * (math.sin(q1) + math.sin(q1 + q2)), 2))
    # ))

def calculateIK_pitch1(wpose: Pose, pitch: float):
    wpose = set_orientation(wpose, pitch)
    z_c = wpose.position.z
    z_l = z_c - center_to_side*math.sin(math.radians(pitch))  
    z_r = z_c + center_to_side*math.sin(math.radians(pitch))  

    alpha = math.radians(85)
    beta_l = math.sin(z_l / (2 * L)) - alpha
    x_l = L * (math.cos(alpha + beta_l))
    q1 = (alpha + beta_l)
    q2 = -beta_l

    alpha = math.radians(85)
    beta_r = math.sin(z_r / (2 * L)) - alpha
    x_r = L * (math.cos(alpha + beta_r))
    q3 = (alpha + beta_r)
    q4 = -beta_r

    joint_position_state=[q1,q2,q1,q2,q3,q4,q3,q4] # stand up principal
    joints_states.position = joint_position_state
    pub.publish(joints_states)

def calculateIK_pitch2(wpose: Pose, pitch: float):
    wpose = set_orientation(wpose, pitch)
    z_c = wpose.position.z
    z_l = z_c - center_to_side*math.sin(math.radians(pitch))  
    z_r = z_c + center_to_side*math.sin(math.radians(pitch))  


    x_c = wpose.position.x
    x_l = arm_l.position.x + center_to_side*(1 - math.cos(math.radians(pitch)))
    x_r = arm_r.position.x - center_to_side*(1 - math.cos(math.radians(pitch)))

    beta_l  =-math.cos(((x_l**2) + (z_l**2) - 2*(L**2))/(2*(L**2))) # angulo del link1 con respecto al suelo
    alpha_l = math.atan2(z_l,x_l) + math.atan2((math.sin(beta_l)), (1+math.cos(beta_l)))# angulo del link2 con respecto al link1
    q1 = -(alpha_l + beta_l)
    q2 = -beta_l

    beta_r  =-math.cos(((x_r**2) + (z_r**2) - 2*(L**2))/(2*(L**2))) # angulo del link1 con respecto al suelo
    alpha_r = math.atan2(z_r,x_r) + math.atan2((math.sin(beta_r)), (1+math.cos(beta_r)))# angulo del link2 con respecto al link1
    q3 = -(alpha_r + beta_r)
    q4 = -beta_r

    joint_position_state=[q1,q2,q1,q2,q3,q4,q3,q4] # stand up principal
    joints_states.position = joint_position_state
    pub.publish(joints_states)

    print(z_c, z_l, z_r, x_c, x_l, x_r)

def calculateIK_pitch(wpose: Pose, pitch: float):
    wpose = set_orientation(wpose, pitch)
    
    # Obtén el pitch en radianes
    pitch = math.radians(pitch)
    
    # Posición vertical del torso
    z_c = wpose.position.z

    # Desplazamiento vertical de las patas dependiendo del pitch
    z_l = z_c - center_to_side * math.sin(pitch)  # Pierna izquierda
    z_r = z_c + center_to_side * math.sin(pitch)  # Pierna derecha

    # Calcula las posiciones en x de las patas dependiendo del pitch
    x_c = wpose.position.x
    x_l = x_c + center_to_side * (1 - math.cos(pitch))  # Pierna izquierda
    x_r = x_c - center_to_side * (1 - math.cos(pitch))  # Pierna derecha

    # Calcula ángulos para las patas izquierda y derecha usando cinemática inversa
    try:
        beta_l = -math.acos((x_l**2 + z_l**2 - 2 * L**2) / (2 * L**2))  # Angulo del link 1 con respecto al suelo
        alpha_l = math.atan2(z_l, x_l) + math.atan2(math.sin(beta_l), 1 + math.cos(beta_l))  # Angulo del link 2
        q1 = -(alpha_l + beta_l)
        q2 = -beta_l

        beta_r = -math.acos((x_r**2 + z_r**2 - 2 * L**2) / (2 * L**2))  # Angulo del link 1 con respecto al suelo
        alpha_r = math.atan2(z_r, x_r) + math.atan2(math.sin(beta_r), 1 + math.cos(beta_r))  # Angulo del link 2
        q3 = -(alpha_r + beta_r)
        q4 = -beta_r

        # Verificación de límites de las articulaciones
        joint_limits = [-math.pi/2, math.pi/2]  # Ejemplo de límites [-90º, 90º]

        # Aplica límites a las articulaciones
        q1 = max(min(q1, joint_limits[1]), joint_limits[0])
        q2 = max(min(q2, joint_limits[1]), joint_limits[0])
        q3 = max(min(q3, joint_limits[1]), joint_limits[0])
        q4 = max(min(q4, joint_limits[1]), joint_limits[0])

        # Actualiza posiciones de las articulaciones
        joint_position_state = [q1, q2, q1, q2, q3, q4, q3, q4]
        joints_states.position = joint_position_state
        pub.publish(joints_states)
    except ValueError:
        print("Error en cálculos de cinemática inversa. Valores fuera de rango.")
        
    # rospy.logwarn("Beta_l: {}, Alpha_l: {}, q1: {}, q2: {}, (X Z): {}  {}\nBeta_r: {}, Alpha_r: {}, q3: {}, q4: {}, (X Z): {}  {}\n".format(
    #     str(round(math.degrees(beta_l),  2)), 
    #     str(round(math.degrees(alpha_l), 2)), 
    #     str(round(math.degrees(q1), 2)), 
    #     str(round(math.degrees(q2), 2)), 
    #     str(round(0.4 * (math.cos(q1) + math.cos(q1 + q2)), 4)), 
    #     str(round(0.4 * (math.sin(q1) + math.sin(q1 + q2)), 4)), 
    #     str(round(math.degrees(beta_r),  2)), 
    #     str(round(math.degrees(alpha_r), 2)), 
    #     str(round(math.degrees(q3), 2)), 
    #     str(round(math.degrees(q4), 2)), 
    #     str(round(0.4 * (math.cos(q3) + math.cos(q3 + q4)), 4)), 
    #     str(round(0.4 * (math.sin(q3) + math.sin(q3 + q4)), 4))
    # ))


if __name__ == "__main__":
    print()
    rospy.sleep(0.5)

    calculateIK_floor(wpose)
    rospy.sleep(2.5)


    while not rospy.is_shutdown():

        number = int(input ("Enter number: "))

        if (number==1):
            # Home
            wpose.position.z    = 0.6
            wpose.orientation.x = 0
            wpose.orientation.y = 0
            wpose.orientation.z = 0
            wpose.orientation.w = 1

            calculateIK_floor(wpose)
            rospy.sleep(1)

        elif (number==2):
            # Up
            wpose.position.z += 0.1
            calculateIK_floor(wpose)
            rospy.sleep(1)

        elif (number ==3):
            # Down
            wpose.position.z -= 0.1
            calculateIK_floor(wpose)
            rospy.sleep(1)

        elif (number ==4):
            calculateIK_pitch2(wpose, 100)
            rospy.sleep(1)

        elif (number ==5):
            calculateIK_pitch1(wpose, -10)
            rospy.sleep(1)

        else:
            pass