[![ROS](http://www.ros.org/wp-content/uploads/2013/10/rosorg-logo1.png)](http://www.ros.org/)

<h1 style="border:none"> RISE Quadruped Robot Simulation and Manipulation Package </h1>
&copy; 2019, Francisco Yumbla

<hr>

## 1. How to Install

### 1.1. System Requirements

This package is written an tested on **Ubuntu 18.04 + ROS Melodic** environment. Dependencies are also for this environment.

V-REP must be installed in advance.

### 1.2. Dependencies Prerequisites

<!-- There are a number of dependencies in this package, since the ABB robot is operated by ROS-Industrial package. Please install all the packages listed below in your Ubuntu PC, in the given order. These packages can be installed by `apt` package manager.

* ros-kinetic-rosserial-arduino
* ros-kinetic-rosserial

Dont forget to clone 

'git clone https://github.com/ros-drivers/rosserial.git'

Now,Extract the metapackage `Quadruped_Foots` into `${ros_workspace}/src`. `catkin_make` your workspace. -->


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


### 3.2. V-REP Simulation

1. V-REP execution (Since simulation is performed with vrep remote api, roscore must be executed first)

2. Run the modules needed for the demo with roslaunch
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


instalar todo esto para el joistick
sudo pip install ds4drv
sudo chmod 666 /dev/hidraw4
https://github.com/naoki-mizuno/ds4_driver.git


roslaunch quadruped_master joystick.launch
roslaunch quadruped_communication quadruped_communication.launch


./vrep.sh

