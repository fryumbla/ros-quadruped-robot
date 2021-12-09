[![ROS](http://www.ros.org/wp-content/uploads/2013/10/rosorg-logo1.png)](http://www.ros.org/)

<h1 style="border:none"> RISE Quadruped Robot Simulation and Manipulation Package </h1>
&copy; 2019, Francisco Yumbla

<hr>

## 1. How to Install

### 1.1. System Requirements

This package is written an tested on **Ubuntu 20.04 + ROS Noetic** environment. Dependencies are also for this environment.

#### 1.1.1 Desktop 

Install ROS in your Ubuntu 20.04 computer (follow this tutorials): http://wiki.ros.org/noetic/Installation/Ubuntu

Create a ROS Workspace: ~/catkin_ws

### 1.2. Dependencies Prerequisites

Please install all the packages listed below in your Ubuntu PC or Jetson Nano, in the given order.

* ros-noetic-moveit

### 1.3 Build

Extract the metapackage `ros-quadruped-robot` into `${ros_workspace}/src`. and `catkin_make` your workspace.
```
git clone --recursive https://github.com/fryumbla/ros-quadruped-robot.git

catkin_make
```


## 2. Structure of Package

To be updated...


## 3. How to Use

### 3.1. Gazebo Simulation

1. Gazebo execution (Since simulation is performed with vrep remote api, roscore must be executed first)
   ```
   roslaunch quadruped_description gazebo.launch
   ```

2. Run the modules needed for the demo with roslaunch
   ```
   rosrun quadruped_master communication_gazebo.py 
   ```
    ```
   rosrun quadruped_master movement.py
   ```


### 3.2. CoppeliaSim Simulation

1. CoppeliaSim execution (Open the scene: 
ros-quadruped-robot/quadruped_vrep/scenes/
), and play

2. Run the modules needed for the demo with roslaunch and the movement program
   ```
   roslaunch quadruped_vrep vrep.launch
   ```
    ```
   rosrun quadruped_master movement.py
   ```


<!-- ### 3.2. Sensors

You can use the connect to the  controller. Type `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1` in the terminal, and you will get a correct port `/dev/ttyACM1`.
You can type ` rosrun foot_pressure_sensor sub_pressure.py` in the terminal. Please be careful when using the program.
this create a topic

* `/Foots_Touch`: move to designated pose 
* `/bolean_foots`: move to designated pose, in straight path. -->
<!-- 

instalar todo esto para el joistick
sudo pip install ds4drv
sudo chmod 666 /dev/hidraw4
https://github.com/naoki-mizuno/ds4_driver.git


roslaunch quadruped_master joystick.launch
roslaunch quadruped_communication quadruped_communication.launch


./vrep.sh -->

