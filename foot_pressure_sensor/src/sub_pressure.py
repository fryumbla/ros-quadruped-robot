#!/usr/bin/env python


import rospy 
import numpy as np
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from rosserial_arduino.msg import Adc


class pressure:
    def __init__(self):
        self.sub=rospy.Subscriber('Foots_Touch', Adc, self.callback)
        self.pub=rospy.Publisher('bolean_foots',Int32MultiArray)
        self.f1=0
        self.f2=0
        self.f3=0
        self.f4=0
        global A
        A=np.array([0,0,0,0],Int32MultiArray)
        

    def callback(self, msg):
        self.f1=msg.adc0
        self.f2=msg.adc1
        self.f3=msg.adc2
        self.f4=msg.adc3
        
        if self.f1 > 255:
            print("foot one is touch")
            A[0]=1
        else:
            print("dont touch one")
        if self.f2 > 255:
            print("foot two is touch")
            A[1]=1
        else:
            print("dont touch two")
        if self.f3 > 255:
            print("foot three is touch")
        else:
            print("dont touch three")
        if self.f4 > 255:
            print("foot four is touch") 
        else:
            print("dont touch four") 
               
                                    
    def get_x(self):
        return A 


def main():
    while not rospy.is_shutdown():
        
        rospy.init_node('measure_pressure') 
        pr = pressure()

        print(pr.get_x())
        

        rospy.spin()

if __name__ == "__main__":
    main()