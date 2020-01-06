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
ADDR_PRO_VELOCITY_LIMIT     = 44
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_LED                = 65
ADDR_PRO_POSITION_D_GAIN    = 80
ADDR_PRO_POSITION_I_GAIN    = 82
ADDR_PRO_POSITION_P_GAIN    = 84

ADDR_PRO_GOAL_POSITION      = 116

#ONLY READ
ADDR_PRO_PRESENT_POSITION   = 132






AX_TORQUE_ENABLE           = 24               # Control table address is different in Dynamixel model
AX_LED                     = 25                 

AX_GOAL_POSITION           = 30             #It is a position value of destination. 0 ~ 1,023 (0x3FF) is available. 
                                            #The unit is 0.29°. If Goal Position is out of the range, 
                                            #Angle Limit Error Bit (Bit 1) of Status Packet is returned as ‘1’ and Alarm is triggered as set in Alarm LED/Shutdown.
AX_MOVING_SPEED            = 32
AX_TORQUE_LIMIT            = 34

#ONLY READ
AX_PRESENT_POSITION        = 36
AX_PRESENT_SPEED           = 38
AX_PRESENT_LOAD            = 40
AX_PRESENT_VOLTAGE         = 42
AX_PRESENT_TEMPERATURE     = 43

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
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler0, i, AX_TORQUE_ENABLE, 0)
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
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, i, AX_TORQUE_ENABLE, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel: ",i," has been successfully connected")



def torque(portHandler, order):
    for i in DXL_ID1:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, AX_TORQUE_ENABLE, order)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            if order == 1:
                print("Torque of Motor ",i," is on")
            else:
                print("Torque of Motor ",i," is off")

def ini_position():
    for i in DXL_ID:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, i, AX_GOAL_POSITION, 512)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

def read_positions():
        # Read present position
    joint_position=[0,0,0,0]
    for i in DXL_ID1:
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler1, i, AX_PRESENT_POSITION)#AX_PRESENT_POSITION
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        joint_position[i-1]=dxl_present_position


    return joint_position

def read_tempeture():
    # Read present temperature
    temperature=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    for i in DXL_ID:
        dxl_present_temperature, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, i, AX_PRESENT_TEMPERATURE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        temperature[i-1]=dxl_present_temperature
    return temperature
    

def convertRadian2Value(radian):
    value = 0
    zero_position = (1023 + 0)/2
    # 1.570796327 pi  the angle in the dynamixel motor is 0 to 300 degree 150 degree represent 2.61799 radians
    if (radian > 0):
        value = (int)(radian * (1023 - zero_position) / 2.61799) + zero_position
    elif (radian < 0):
        value = (int)(-radian * (0 - zero_position) / 2.61799) + zero_position
    else:
        value = zero_position

    return value


def convertValue2Radian(value):

    radian = 0.0
    zero_position = (1023 + 0)/2

    if (value > zero_position):
        radian = (float)(value - zero_position) * 1.5708 / (float)(1023 - zero_position)
    elif (value < zero_position):
        radian = (float)(value - zero_position) * 0.1 / (float)(0 - zero_position)
    return radian




def callback(data):
      
    for i in DXL_ID:     
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, i, AX_GOAL_POSITION, convertRadian2Value(data.position[i-1]))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    joint_position_state=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    pub = rospy.Publisher('current_joint_states', JointState, queue_size=10)
    joints_states = JointState()
    joints_states.header = Header()
    joints_states.header.stamp = rospy.Time.now()
    joints_states.name = ['joint_1', 'joint_2', 'joint_3','joint_4', 'joint_5', 'joint_6', 'joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12', 'joint_13','joint_14', 'joint_15', 'joint_16', 'joint_17','joint_18', 'joint_19', 'joint_20', 'joint_21']
    
    joint_position=read_positions()
    
    for i in range(0,20):
        joint_position_state[i]=convertValue2Radian(joint_position[i])

    print(read_tempeture())

    joints_states.position = joint_position_state
    joints_states.velocity = []
    joints_states.effort = []
    pub.publish(joints_states)
    

def main():


    comunication0()
    comunication1()
    
    torque(portHandler0,1)
    torque(portHandler1,1)
    
    # ini_position()
    # torque(0)
    
    rospy.init_node('joint_publisher',anonymous=True)

    joint_pub = rospy.Publisher('joint_states',JointState,queue_size=1)
    joint_instance = JointState()
    joint_instance.name.append("front_left_joint1")
    joint_instance.name.append("front_left_joint2")
    joint_instance.name.append("front_right_joint1")
    joint_instance.name.append("front_right_joint2")
    joint_instance.name.append("back_left_joint1")
    joint_instance.name.append("back_left_joint2")
    joint_instance.name.append("back_right_joint1")
    joint_instance.name.append("back_right_joint2")
    joint_instance.position.append(0.0)
    joint_instance.position.append(0.0)
    joint_instance.position.append(0.0)
    joint_instance.position.append(0.0)
    joint_instance.position.append(0.0)
    joint_instance.position.append(0.0)
    joint_instance.position.append(0.0)
    joint_instance.position.append(0.0)
           
    # while not rospy.is_shutdown():
        
    #     joint_position_state=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

    #     pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    #     rospy.init_node("state_joints")
    #     # rate = rospy.Rate(1000000) # 10hz
    #     rate = rospy.Rate(10) # 10hz   
    #     joints_states = JointState()
    #     joints_states.header = Header()
    #     joints_states.header.stamp = rospy.Time.now()
    #     joints_states.name = ['joint_1', 'joint_2', 'joint_3','joint_4', 'joint_5', 'joint_6', 'joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12', 'joint_13','joint_14', 'joint_15', 'joint_16', 'joint_17','joint_18', 'joint_19', 'joint_20', 'joint_21']
        
    #     joint_position=read_positions()
        
    #     for i in range(0,20):
    #         joint_position_state[i]=convertValue2Radian(joint_position[i])


    #     joints_states.position = joint_position_state
    #     joints_states.velocity = []
    #     joints_states.effort = []
    #     pub.publish(joints_states)
        # rate.sleep()   

    while not rospy.is_shutdown():

        # rospy.Subscriber('/joint_states', JointState, callback)

        # joint_current_position=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        # joint_current_position=read_positions()
        # print 'angle'
        # for i in range(0,21):
        #     print i+1, joint_current_position[i]
        #     print joint_current_position[i]

        rate = rospy.Rate(10) # 10hz
        rospy.spin()







if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        portHandler.closePort()
        pass
    



    
        



    
