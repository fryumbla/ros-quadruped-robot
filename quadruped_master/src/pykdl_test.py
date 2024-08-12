#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import roslib
import os
import PyKDL as kdl
import os
from kdl_parser_py.urdf import treeFromFile
from trac_ik_python.trac_ik import IK

# Uses Dynamixel SDK library
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math

def calculate_jacobian(current_positions, chain):

    jt_positions = kdl.JntArray(NUM_MOTORS)
    for i in range(NUM_MOTORS):
        jt_positions[i] = current_positions[i]

    jacobian = kdl.Jacobian(NUM_MOTORS)
    jac_solver = kdl.ChainJntToJacSolver(chain)
    jac_solver.JntToJac(jt_positions, jacobian)

    jacobian_matrix = []
    for i in range(jacobian.rows()):
        row = []
        for j in range(jacobian.columns()):
            row.append(round(jacobian[i, j],4))
        jacobian_matrix.append(row)

    return jacobian_matrix

def calculate_mass_matrix(current_positions, chain):

    # Crear el vector de posiciones articulares
    jt_positions = kdl.JntArray(NUM_MOTORS)
    for i in range(NUM_MOTORS):
        jt_positions[i] = current_positions[i]

    # Crear la matriz de masas y el solver
    mass_matrix = kdl.JntSpaceInertiaMatrix(NUM_MOTORS)
    dyn_kdl = kdl.ChainDynParam(chain, kdl.Vector.Zero())
    dyn_kdl.JntToMass(jt_positions, mass_matrix)

    # Convertir la matriz de masas a una matriz de Python
    mass_matrix_py = []
    for i in range(mass_matrix.rows()):
        row = []
        for j in range(mass_matrix.columns()):
            row.append(round(mass_matrix[i, j],4))
        mass_matrix_py.append(row)

    return mass_matrix_py

def gravity_compensation(current_positions, chain):

    grav_vector = kdl.Vector(0, 0, -9.81)  # relative to kdl chain base link
    dyn_kdl = kdl.ChainDynParam(chain, grav_vector)
    jt_positions = kdl.JntArray(NUM_MOTORS)
    for i in range(NUM_MOTORS):
        jt_positions[i] = current_positions[i]

    grav_matrix = kdl.JntArray(NUM_MOTORS)
    dyn_kdl.JntToGravity(jt_positions, grav_matrix)

    gravity_compensating_jt_torques = [grav_matrix[i] for i in range(grav_matrix.rows())]

    return gravity_compensating_jt_torques    


def move_to_target(state_position: JointState):

    # Cargar el URDF y crear el árbol de KDL
    success, kdl_tree = treeFromFile(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'archie_description', 'urdf','manipulator.urdf'))
    chain = kdl_tree.getChain("base_link", "link_6")

    # Calcula los torques de gravedad para la posición actual
    gravity_torques = gravity_compensation(motor_positions, chain)

    # Calcula los torques adicionales necesarios para moverse hacia la posición objetivo
    position_error = np.array(state_position.position) - np.array(current_positions)

    k_p = np.array([3, 3.5, 2.5, 2, 2, 2])
    c_p = np.array([0.2, 0.6, 0.15, 0.4, 0.1, 0.1])

    error_torques = (position_error*k_p)
    damp_torques  = ((np.array(motor_velocities, float)*(2*math.pi/60))*c_p) #primero convertimos vel a rad/s

    print("\n","Jacobian Matrix:")
    jacobian_matrix = calculate_jacobian(state_position.position, chain)
    print(jacobian_matrix[0])
    print(jacobian_matrix[1])
    print(jacobian_matrix[2])
    print(jacobian_matrix[3])
    print(jacobian_matrix[4])
    print(jacobian_matrix[5])

    print("\n","Mass Matrix:")
    mass_matrix = calculate_mass_matrix(state_position.position, chain)
    print(mass_matrix[0])
    print(mass_matrix[1])
    print(mass_matrix[2])
    print(mass_matrix[3])
    print(mass_matrix[4])
    print(mass_matrix[5])
    
    total_pwm = (gravity_torques + error_torques - damp_torques)*np.array([885/1.8, 885/1.8, 885/1.8, 885/1.8, 885/1.4, 885/1.4])
    for id in range(len(total_pwm)):
        total_pwm[id] = (round(total_pwm[id]) if (total_pwm[id] < 700 and total_pwm[id] > -700) else
                        (700      if total_pwm[id] > 700 else (-700)))
    total_pwm = np.array(total_pwm,int)
    
    motor_efforts = total_pwm/np.array([885/1.8, 885/1.8, 885/1.8, 885/1.8, 885/1.4, 885/1.4])

    joint_state_publisher(motor_positions, motor_velocities, motor_efforts)
    set_sync_pwm(np.array(total_pwm))

    rospy.logwarn(f"PWM: {total_pwm}")
    rospy.logwarn(f"Vel: {motor_velocities}")
    rospy.logwarn(f"Par: {motor_efforts}")
    rospy.logwarn(f"Err: {position_error*180/math.pi}")
    
    motor = MotorData()
    motor.position = motor_positions
    motor.error    = position_error
    motor.velocity = motor_velocities
    motor.effort   = motor_efforts

    motor_state_pub.publish(motor)
    print("=====================================================================================")


def joint_state_publisher(motor_positions, motor_velocities, motor_efforts):
    joints_states = JointState()
    joints_states.header = Header()
    joints_states.header.stamp = rospy.Time.now()
    joints_states.name = ['joint_'+str(id) for id in range(NUM_MOTORS)]
    
    #Publish the new joint state
    joints_states.position = motor_positions
    joints_states.velocity = motor_velocities
    joints_states.effort = motor_efforts
    joint_state_pub.publish(joints_states)


if __name__ == '__main__':

    rospy.init_node("test_pykdl")
    r =rospy.Rate(10) # 10hz
  
    #Publish current robot state
    joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    #subGoalState    = rospy.Subscriber('/joint_goals', JointState, callback = move_to_target, queue_size= 5)
    
    # Crear el árbol KDL vacío
    tree = kdl.Tree("body")

    # Añadir la articulación front_right_joint1
    f_joint1 = kdl.Joint(kdl.Joint.RotY)
    f_frame1 = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.11, -0.05, 0))
    tree.addSegment(kdl.Segment("front_right_u", f_joint1, f_frame1), "body")

    # Añadir la articulación front_right_joint2
    f_joint2 = kdl.Joint(kdl.Joint.RotY)
    f_frame2 = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.4, -0.005, 0))
    tree.addSegment(kdl.Segment("front_right_d", f_joint2, f_frame2), "front_right_u")

    # Crear la cadena de la pata desde el end effector (punto final) hasta el cuerpo
    chain = tree.getChain("body", "front_right_d")

    # Definir los nombres de los frames (enlaces) para IK
    ik_solver = IK("body", "front_right_d")

    # Ejemplo: Resolver IK para un objetivo deseado en la pata
    x, y, z = 0, 0, 0  # Coordenadas del objetivo
    qinit = [0.76, 1.17]  # Ángulos iniciales aproximados para las dos articulaciones

    # Resolver la IK
    solution = ik_solver.get_ik(qinit, x, y, z, 1.57, 0, 1.57, 1.57)
    if solution:
        print("Ángulos de articulación:", solution)
    else:
        print("No se encontró solución")