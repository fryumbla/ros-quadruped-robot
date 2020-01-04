extern "C" {
    #include "/home/francisco/quadruped_ws/src/quadruped_robot/quadruped_vrep/remoteApi/extApi.h"
    
}

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
  if(simxGetObjectHandle(a,(const simxChar*) b,(simxInt *) &c,(simxInt) simx_opmode_oneshot_wait))
  {
    cout << "no joint " << d << " found" << std::endl;
  }
  else
  {
    return c;
    cout << "joint " << d << " found"  << std::endl;
  }

}

void joint_callback(const sensor_msgs::JointState& data)
{
  pub_msg.name=data.name;
  pub_msg.position = data.position;


  int joint_1_handle=0;
  int joint_2_handle=0;
  int joint_3_handle=0;
  int joint_4_handle=0;
  int joint_5_handle=0;
  int joint_6_handle=0;
  int joint_7_handle=0;
  int joint_8_handle=0;


  joint_1_handle=found(clientID,"front_right_joint1",joint_1_handle,1);
  joint_2_handle=found(clientID,"front_left_joint1",joint_2_handle,2);
  joint_3_handle=found(clientID,"back_left_joint1",joint_3_handle,3);
  joint_4_handle=found(clientID,"back_right_joint1",joint_4_handle,4);
  joint_5_handle=found(clientID,"front_right_joint2",joint_5_handle,5);
  joint_6_handle=found(clientID,"front_left_joint2",joint_6_handle,6);
  joint_7_handle=found(clientID,"back_left_joint2",joint_7_handle,7);
  joint_8_handle=found(clientID,"back_right_joint2",joint_8_handle,8);


  // cout << joint_1_handle  << std::endl;
  // cout << joint_2_handle << std::endl;
  // cout << joint_3_handle << std::endl;
  // cout << joint_4_handle << std::endl;
  // cout << joint_5_handle << std::endl;
 



  //simxGetJointPosition(clientID, joint_1_handle, simx_opmode_streaming);
  simxSetJointTargetPosition(clientID, (simxInt) joint_1_handle, data.position.at(0), simx_opmode_oneshot);
  simxSetJointTargetPosition(clientID, (simxInt) joint_2_handle, data.position.at(1), simx_opmode_oneshot);
  simxSetJointTargetPosition(clientID, (simxInt) joint_3_handle, data.position.at(2), simx_opmode_oneshot);
  simxSetJointTargetPosition(clientID, (simxInt) joint_4_handle, data.position.at(3), simx_opmode_oneshot);
  simxSetJointTargetPosition(clientID, (simxInt) joint_5_handle, data.position.at(4), simx_opmode_oneshot);
  simxSetJointTargetPosition(clientID, (simxInt) joint_6_handle, data.position.at(5), simx_opmode_oneshot);
  simxSetJointTargetPosition(clientID, (simxInt) joint_7_handle, data.position.at(6), simx_opmode_oneshot);
  simxSetJointTargetPosition(clientID, (simxInt) joint_8_handle, data.position.at(7), simx_opmode_oneshot);



  //cout << pub_msg.name.at(0) << std::endl;
}

int main(int argc, char **argv) 
{
  string serverIP = "127.0.0.1";
  int serverPort = 19999;

  clientID=simxStart((simxChar*)serverIP.c_str(),serverPort,true,true,2000,5);
 
  if (clientID!=-1)
  {
    cout << "Servidor conectado!" << std::endl;
    
    ros::init(argc, argv, "vrep_control");
    ros::NodeHandle nh = ros::NodeHandle();
    
    ros::Subscriber sub = nh.subscribe("/joint_states", 2000, joint_callback);

    // Waits for simulation time update.
    ros::Time last_ros_time_;
    bool wait = true;

    ros::spin();
    simxFinish(clientID); // fechando conexao com o servidor
    cout << "Conexao fechada!" << std::endl;
  }
  else
    cout << "Problemas para conectar con servidor!" << std::endl;
  return 0;

  
}


