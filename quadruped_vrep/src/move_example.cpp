#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{

    if (1==1)
    {
        //Initializen the ros
        ros::init(argc, argv, "joint_state_publisher");

        //Declare the node handle
        ros::NodeHandle node;

        //Decleare a joint state publisher
        ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("/joint_states",1);
        ros::Rate loop_rate(100);

        //Define the joint state
        sensor_msgs::JointState joint_state;

        

        joint_state.name.push_back("arm_1_joint");
        joint_state.name.push_back("arm_2_joint");
        joint_state.name.push_back("arm_3_joint");
        joint_state.name.push_back("arm_4_joint");
        joint_state.name.push_back("arm_5_joint");
        joint_state.name.push_back("arm_6_joint");

        joint_state.position.resize(6);
        for(size_t i = 0; i < 6; i++)
            joint_state.position[i] = 0.5;
        std::cout<<joint_state<<std::endl;



        //joint_state.position.push_back(0.0);


        while(ros::ok()){
            joint_state.header.stamp= ros::Time::now();
            //command the robot to move
            joint_pub.publish(joint_state);
            ros::spinOnce();
            loop_rate.sleep();
        }
        
        return 0;
    }

}