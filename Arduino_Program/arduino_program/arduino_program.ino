/*
 * rosserial publisher presure sensor in the foot quadrupe
 */
#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>


//set up the ros node and publisher
ros::NodeHandle nh;
std_msgs::Float32 pressure_msg;
ros::Publisher pub_pressure("pressure", &pressure_msg);


int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider

void setup() {
  // We'll send debugging information via the Serial monitor
  nh.initNode();
  nh.advertise(pub_pressure);

  
    
}
 
void loop() {
  
  fsrReading = analogRead(fsrPin);  
  pressure_msg.data= fsrReading;
  pub_pressure.publish(&pressure_msg);
  nh.spinOnce();

} 
