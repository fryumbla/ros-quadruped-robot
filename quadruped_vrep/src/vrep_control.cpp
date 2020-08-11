#include "../remoteApi/extApi.h"
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

using namespace std;

int clientID;

sensor_msgs::JointState pub_msg;

int found(int a,char* b,int c,int d)
{
  if(simxGetObjectHandle(a,(const simxChar*) b,(simxInt *) &c,(simxInt) simx_opmode_oneshot_wait)){
    cout << "no joint " << d << " found" << std::endl;
  }
  else{
    return c;
    cout << "joint " << d << " found"  << std::endl;
  }
}

void joint_callback(const sensor_msgs::JointState& data)
{
  pub_msg.name=data.name;
  pub_msg.position = data.position;

  std::vector<char*> joints;
  for (int i=0;i<data.name.size(); ++i){
    char *cstr = new char[data.name.at(i).length() + 1];
    strcpy(cstr, data.name.at(i).c_str());
    joints.push_back(cstr);
  }

  std::vector<int> joint_handle;
  for (int i = 0; i < data.name.size(); i++)
  {
    joint_handle.push_back(0);
  }
  for (int i=0;i<data.name.size(); ++i){
    joint_handle[i]=found(clientID,joints[i],joint_handle[i],i+1);
  }
  for (int i=0;i<data.name.size(); ++i){
    simxSetJointTargetPosition(clientID, (simxInt) joint_handle[i], data.position.at(i), simx_opmode_oneshot);
  }

}

int main(int argc, char **argv) 
{
  string serverIP = "127.0.0.1";
  int serverPort = 19999;

  clientID=simxStart((simxChar*)serverIP.c_str(),serverPort,true,true,2000,5);
  
  if (clientID!=-1)
  {
    cout << "Server conected!" << std::endl;
    
    ros::init(argc, argv, "vrep_communication");
    ros::NodeHandle nh = ros::NodeHandle();
    
    ros::Subscriber sub = nh.subscribe("/joint_states", 2000, joint_callback);

    // Waits for simulation time update.
    ros::Time last_ros_time_;
    bool wait = true;

    ros::spin();
    simxFinish(clientID); // fechando conexao com o servidor
    cout << "Connection over!" << std::endl;
  }
  else
    cout << "Server connector problem!" << std::endl;
  return 0;
}


