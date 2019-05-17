#!/usr/bin/env python


import rospy 
import numpy as np
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from rosserial_arduino.msg import Adc


class pressure:
    def __init__(self):
        self.sub=rospy.Subscriber('bolean_foots', Adc, self.callback)
               

    def callback(self, msg):
        self.f1=msg.adc0
                     
                                    
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