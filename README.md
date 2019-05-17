<h1 style="border:none"> RISE Quadruped Robot Package </h1>
&copy; 2018, Francisco Yumbla

<hr>

## 1. How to Install

### 1.1. System Requirements

This package is written an tested on **Ubuntu 16.04 + ROS Kinetic** environment. Dependencies are also for this environment.

### 1.2. Dependencies Prerequisites

There are a number of dependencies in this package, since the ABB robot is operated by ROS-Industrial package. Please install all the packages listed below in your Ubuntu PC, in the given order. These packages can be installed by `apt` package manager.

* ros-kinetic-rosserial-arduino
* ros-kinetic-rosserial

Dont forget to clone 

'git clone https://github.com/ros-drivers/rosserial.git'

Now,Extract the metapackage `Quadruped_Foots` into `${ros_workspace}/src`. `catkin_make` your workspace.


## 2. Structure of Package

To be updated...


## 3. How to Use

### 3.1. Serial Arduino 

You can use the connect to the  controller. Type `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1` in the terminal, and you will get a correct port `/dev/ttyACM1`.

### 3.2. Convert the analog singnal to boolean

You can type ` rosrun foot_pressure_sensor sub_pressure.py` in the terminal. Please be careful when using the program.
this create a topic

* `/Foots_Touch`: move to designated pose 
* `/bolean_foots`: move to designated pose, in straight path.
