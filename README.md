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

Now,Extract the metapackage `Quadruped_Foots` into `${ros_workspace}/src`. `catkin_make` your workspace.


## 2. Structure of Package

To be updated...


## 3. How to Use

### 3.1. CLI Controller

You can use the CLI controller. Type `rosrun rise_assembler assembler_manual_controller` in the terminal, and you will get a CLI controller displayed up on your console. To see the list of commands, type `h` and `Enter`.

### 3.2. GUI Controller

You can use the GUI controller. Type `rosrun rise_assembler assembler_gui_controller` in the terminal, and then the GUI console will pop out. This program is not complete yet, so it might be prone to errors. Please be careful when using the program.

### 3.3. Path-Planning APIs

The script `assembler_controller.py` provides a convenient method to plan paths. One should give the pose of end-effector in the following form; [x, y, z, roll, pitch, yaw].

* `move_to_pose()`: move to designated pose 
* `move_by_cartesian_path()`: move to designated pose, in straight path.
* `rotate_joint()`: rotate specific joint by desiged angle. This method gets the angles of all 6 joints, add the given value to specified joint, and then makes the robot go to that configuration. Therefore, the joints that are not designated to move might move by a little angle, due to controller errors.