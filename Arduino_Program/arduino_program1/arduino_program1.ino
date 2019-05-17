/*
 * rosserial publisher presure sensor in the foot quadrupe
 */
#define USE_USBCON
#include <ros.h>
#include <rosserial_arduino/Adc.h>



//set up the ros node and publisher
ros::NodeHandle nh;
rosserial_arduino::Adc pressure_msg;
ros::Publisher pub_pressure("Foots_Touch", &pressure_msg);


int foot1 = 0;     // the FSR and 10K pulldown are connected to a0
int foot2 = 1;
int foot3 = 2;
int foot4 = 3;

void setup() {
  // We'll send debugging information via the Serial monitor
  nh.initNode();
  nh.advertise(pub_pressure);
    
}
 
void loop() {
  
  pressure_msg.adc0 = analogRead(foot1);
  pressure_msg.adc1 = analogRead(foot2);  
  pressure_msg.adc2 = analogRead(foot3);  
  pressure_msg.adc3 = analogRead(foot4);  
  
  pub_pressure.publish(&pressure_msg);
  
  nh.spinOnce();

} 
