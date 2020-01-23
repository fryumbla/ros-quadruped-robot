#!/usr/bin/env python
# -*- coding: utf-8 -*-

# With this program we read and write the Dynamixel Motores using SDK 
import rospy
import os

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library


# Control table address
#READ AND WRITE
ADDR_PRO_ACCELERATION_LIMIT = 40
ADDR_PRO_VELOCITY_LIMIT     = 44
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_LED                = 65
ADDR_PRO_POSITION_D_GAIN    = 80
ADDR_PRO_POSITION_I_GAIN    = 82
ADDR_PRO_POSITION_P_GAIN    = 84

ADDR_PRO_GOAL_VELOCITY      = 104
ADDR_PRO_PROFILE_ACCELERATION = 108
ADDR_PRO_PROFILE_VELOCITY   = 112
ADDR_PRO_GOAL_POSITION      = 116

#ONLY READ
ADDR_PRO_PRESENT_VELOCITY   = 128
ADDR_PRO_PRESENT_POSITION   = 132
ADDR_PRO_PRESENT_TEMPERATURE= 146

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel
# Default setting
DXL_ID0                      = [1,2,5,6]                 # Dynamixel ID : 1
DXL_ID1                      = [3,4,7,8]
DXL_ID                      = [1,2,3,4,5,6,7,8]

# BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
BAUDRATE                    = 57600


DEVICENAME0                 = '/dev/ttyUSB0'    # Check which port is being used on your controller
DEVICENAME1                 = '/dev/ttyUSB1'    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler0 = PortHandler(DEVICENAME0)
portHandler1 = PortHandler(DEVICENAME1)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def comunication0():
    # Open port
    if portHandler0.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler0.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()


    for i in DXL_ID0:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler0, i, ADDR_PRO_TORQUE_ENABLE, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel: ",i," has been successfully connected")

def comunication1():
    # Open port
    if portHandler1.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler1.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
    

    for i in DXL_ID1:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, i, ADDR_PRO_TORQUE_ENABLE, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel: ",i," has been successfully connected")



def torque(DXL_ID,portHandler, order):
    for i in DXL_ID:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_PRO_TORQUE_ENABLE, order)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            if order == 1:
                print("Torque of Motor ",i," is on")
            else:
                print("Torque of Motor ",i," is off")

def pid_gain_position_loop():
    # set_P_Gain = 500   
    # set_I_Gain = 100     
    # set_D_Gain = 4700 
    set_P_Gain = 2000
    set_I_Gain = 10   
    set_D_Gain = 10
    for i in DXL_ID0:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler0, i, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler0, i, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler0, i, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel: ",i," has been successfully PID configuration")
    for i in DXL_ID1:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, i, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, i, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, i, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel: ",i," has been successfully PID configuration")

def pid_gain_position_loop():
    set_A_l = 80
    set_V_l = 80

    set_A_PRFL = 10
    set_V_PRFL = 10
    for i in DXL_ID0:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler0, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT, int(set_V_l))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler0, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT, int(set_A_l))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler0, DXL1_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler0, DXL1_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))    
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel: ",i," has been successfully velocity and acceleration configuration")
    for i in DXL_ID1:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT, int(set_V_l))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT, int(set_A_l))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL1_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL1_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel: ",i," has been successfully velocity and acceleration configuration")

def read_positions():
    # Read present position
    joint_position=[0,0,0,0,0,0,0,0]
    # Read Dynamixel#1 present position
    dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler0, 1, ADDR_PRO_PRESENT_POSITION)
    # Read Dynamixel#2 present position
    dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler0, 2, ADDR_PRO_PRESENT_POSITION)
    # Read Dynamixel#3 present position
    dxl3_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler1, 3, ADDR_PRO_PRESENT_POSITION)
    # Read Dynamixel#4 present position
    dxl4_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler1, 4, ADDR_PRO_PRESENT_POSITION)
    # Read Dynamixel#5 present position
    dxl5_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler0, 5, ADDR_PRO_PRESENT_POSITION)
    # Read Dynamixel#6 present position
    dxl6_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler0, 6, ADDR_PRO_PRESENT_POSITION)
    # Read Dynamixel#7 present position
    dxl7_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler1, 7, ADDR_PRO_PRESENT_POSITION)
    # Read Dynamixel#8 present position
    dxl8_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler1, 8, ADDR_PRO_PRESENT_POSITION)

    #1 degree ~ 90
    offset1 = -410
    offset2 = -80
    offset3 = 167
    offset4 = -300
    offset5 = 0
    offset6 = -300
    offset7 = -168
    offset8 = -120

    theta1 = (20475.0 + offset1 -dxl1_present_position)*(15.0/120.0)*(360.0/4095.0)-90.0
    theta2 = (16380.0 + offset2 -dxl2_present_position)*(15.0/120.0)*(360.0/4095.0)
    theta3 = ((dxl3_present_position-offset3)*(15.0/120.0)*(360.0/4095.0))-90.0
    theta4 = ((dxl4_present_position-offset4)*(15.0/120.0)*(360.0/4095.0))-45.0
    theta5 = (20475.0 + offset5 -dxl5_present_position)*(15.0/120.0)*(360.0/4095.0)-90.0
    theta6 = (16380.0 + offset6 -dxl6_present_position)*(15.0/120.0)*(360.0/4095.0)
    theta7 = ((dxl7_present_position-offset7)*(15.0/120.0)*(360.0/4095.0))-90.0
    theta8 = ((dxl8_present_position-offset8)*(15.0/120.0)*(360.0/4095.0))-45.0
    
    print("[1]%.2f\t[2]%.2f\t[3]%.2f\t[4]%.2f\t[5]%.2f\t[6]%.2f\t[7]%.2f\t[8]%.2f " % (dxl1_present_position,dxl2_present_position,dxl3_present_position,dxl4_present_position,dxl5_present_position,dxl6_present_position,dxl7_present_position,dxl8_present_position))

    print("[1]%.2f\t[2]%.2f\t[3]%.2f\t[4]%.2f\t[5]%.2f\t[6]%.2f\t[7]%.2f\t[8]%.2f " % (theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta8))
    
    
    joint_position[0] = theta1*3.1415/180.0
    joint_position[1] = theta2*3.1415/180.0
    joint_position[2] = theta3*3.1415/180.0
    joint_position[3] = theta4*3.1415/180.0
    joint_position[4] = theta5*3.1415/180.0
    joint_position[5] = theta6*3.1415/180.0
    joint_position[6] = theta7*3.1415/180.0
    joint_position[7] = theta8*3.1415/180.0
    
    # dxl1_goal_position = int(round((theta1+45.0) * (120.0/28.0) * (4095.0/360.0)))

    theta1r = joint_position[0]*180/3.1416
    theta2r = joint_position[1]*180/3.1416
    theta3r = joint_position[2]*180/3.1416
    theta4r = joint_position[3]*180/3.1416
    theta5r = joint_position[4]*180/3.1416
    theta6r = joint_position[5]*180/3.1416
    theta7r = joint_position[6]*180/3.1416
    theta8r = joint_position[7]*180/3.1416
    

    dxl1_goal_position = 20475.0 + offset1-((theta1r+90.0)/((15.0/120.0)*(360.0/4095.0)))
    dxl2_goal_position = 16380.0 + offset2-((theta2r)/((15.0/120.0)*(360.0/4095.0)))
    dxl3_goal_position = ((theta3r+90)/((15.0/120.0)*(360.0/4095.0)))+offset3
    dxl4_goal_position = ((theta4r+45)/((15.0/120.0)*(360.0/4095.0)))+offset4
    dxl5_goal_position = 20475.0 + offset5-((theta5r+90.0)/((15.0/120.0)*(360.0/4095.0)))
    dxl6_goal_position = 16380.0 + offset6-((theta6r)/((15.0/120.0)*(360.0/4095.0)))
    dxl7_goal_position = ((theta7r+90)/((15.0/120.0)*(360.0/4095.0)))+offset7
    dxl8_goal_position = ((theta8r+45)/((15.0/120.0)*(360.0/4095.0)))+offset8

    print("[1]%.2f\t[2]%.2f\t[3]%.2f\t[4]%.2f\t[5]%.2f\t[6]%.2f\t[7]%.2f\t[8]%.2f " % (dxl1_goal_position,dxl2_goal_position,dxl3_goal_position,dxl4_goal_position,dxl5_goal_position,dxl6_goal_position,dxl7_goal_position,dxl8_goal_position))
    print()
    
    return joint_position

def callback(data):

    theta1 = data.position[0]*180/3.1416
    theta2 = data.position[0]*180/3.1416
    theta3 = data.position[0]*180/3.1416
    theta4 = data.position[0]*180/3.1416
    theta5 = data.position[0]*180/3.1416
    theta6 = data.position[0]*180/3.1416
    theta7 = data.position[0]*180/3.1416
    theta8 = data.position[0]*180/3.1416
    

    dxl1_goal_position = 20475.0 + offset1-((theta1+90.0)/((15.0/120.0)*(360.0/4095.0)))
    dxl2_goal_position = 16380.0 + offset2-((theta2)/((15.0/120.0)*(360.0/4095.0)))
    dxl3_goal_position = ((theta3+90)/((15.0/120.0)*(360.0/4095.0)))+offset3
    dxl4_goal_position = ((theta4+45)/((15.0/120.0)*(360.0/4095.0)))+offset4
    dxl5_goal_position = 20475.0 + offset5-((theta5+90.0)/((15.0/120.0)*(360.0/4095.0)))
    dxl6_goal_position = 16380.0 + offset6-((theta6)/((15.0/120.0)*(360.0/4095.0)))
    dxl7_goal_position = ((theta7+90)/((15.0/120.0)*(360.0/4095.0)))+offset7
    dxl8_goal_position = ((theta8+45)/((15.0/120.0)*(360.0/4095.0)))+offset8
    
    print("[1]%.2f\t[2]%.2f\t[3]%.2f\t[4]%.2f\t[5]%.2f\t[6]%.2f\t[7]%.2f\t[8]%.2f " % (dxl1_goal_position,dxl2_goal_position,dxl3_goal_position,dxl4_goal_position,dxl5_goal_position,dxl6_goal_position,dxl7_goal_position,dxl8_goal_position))
    print()

    # dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler0, 1, dxl1_goal_position)
    # dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler0, 2, dxl2_goal_position)
    # dxl3_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler1, 3, dxl3_goal_position)
    # dxl4_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler1, 4, dxl4_goal_position)
    # dxl5_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler0, 5, dxl5_goal_position)
    # dxl6_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler0, 6, dxl6_goal_position)
    # dxl7_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler1, 7, dxl7_goal_position)
    # dxl8_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler1, 8, dxl8_goal_position)

      
    # for i in DXL_ID:     
    #     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, i, ADDR_PRO_GOAL_POSITION, convertRadian2Value(data.position[i-1]))
    #     # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL1_ID, ADDR_PRO_GOAL_POSITION, goal1[index])
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))


    # joint_position_state=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    # pub = rospy.Publisher('current_joint_states', JointState, queue_size=10)
    # joints_states = JointState()
    # joints_states.header = Header()
    # joints_states.header.stamp = rospy.Time.now()

    # # joints_states.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1','front_left_joint2', 'back_left_joint1', 'back_left_joint2', 'back_right_joint1','back_right_joint2']
    # joints_states.position = read_positions()
    # joints_states.velocity = []
    # joints_states.effort = []
    # pub.publish(joints_states)
    

def main():


    rospy.init_node("communication")

    comunication0()
    comunication1()
    
    torque(DXL_ID0,portHandler0,1)
    torque(DXL_ID1,portHandler1,1)
    # while not rospy.is_shutdown():
    #     rospy.Subscriber('/joint_states', JointState, callback)
    #     # read_positions()
 
    
           
    while not rospy.is_shutdown():
        
        joint_position_state=[0,0,0,0,0,0,0,0]

        # rospy.init_node("state_joints")
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        # rate = rospy.Rate(1000000) # 10hz
        rate = rospy.Rate(10) # 10hz   
        joints_states = JointState()
        joints_states.header = Header()
        joints_states.header.stamp = rospy.Time.now()
        joints_states.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1','front_left_joint2', 'back_left_joint1', 'back_left_joint2', 'back_right_joint1','back_right_joint2']
        joints_states.position = read_positions()
        joints_states.velocity = []
        joints_states.effort = []
        pub.publish(joints_states)
        rate.sleep()   


    # while not rospy.is_shutdown():

        # rospy.Subscriber('/joint_states', JointState, callback)

    #     # joint_current_position=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    #     # joint_current_position=read_positions()
    #     # print 'angle'
    #     # for i in range(0,21):
    #     #     print i+1, joint_current_position[i]
    #     #     print joint_current_position[i]

    #     rate = rospy.Rate(10) # 10hz
    #     rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        portHandler0.closePort()
        portHandler1.closePort()
        pass
    



    
        



    
