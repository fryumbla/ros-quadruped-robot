#!/usr/bin/env python


import rospy 
import numpy as np
from std_msgs.msg import Float32
from rosserial_arduino.msg import Adc
from foot_pressure_sensor.msg import Num


class pressure:
    def __init__(self):
        self.sub=rospy.Subscriber('Foots_Touch', Adc, self.callback)
        self.pub=rospy.Publisher('bolean_foots', Num, queue_size=10)
        self.f1=0
        self.f2=0
        self.f3=0
        self.f4=0
        self.A=Num()
       
    def callback(self, msg):
        self.f1=msg.adc0
        self.f2=msg.adc1
        self.f3=msg.adc2
        self.f4=msg.adc3
        
        if self.f1 > 255:
            print("foot one is touch")
            self.A.foot1=True
        else:
            #print("dont touch one")
            self.A.foot1=False
        if self.f2 > 255:
            #print("foot two is touch")
            self.A.foot2=True           
        else:
            #print("dont touch two")
            self.A.foot2=False
        if self.f3 > 255:
            #print("foot three is touch")
            self.A.foot3=True
        else:
            #print("dont touch three")
            self.A.foot3=False
        if self.f4 > 255:
            #print("foot four is touch")
            self.A.foot4=True 
        else:
            #print("dont touch four")
            self.A.foot4=False
        self.pub.publish(self.A)
                                    
def main():
    while not rospy.is_shutdown():
        
        rospy.init_node('measure_pressure') 
        pressure()


 
        rospy.spin()

if __name__ == "__main__":
    main()