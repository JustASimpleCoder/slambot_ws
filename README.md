
# 4-Wheel Mecanum Drive SLAM Robot

This repository contains ROS2 packages for exploring SLAM (Simultaneous Localization and Mapping) with a 4-wheel differential mecanum drive robot. The robot is built using:

* Chassis: Basic robot chassis with a 4-wheel mecanum drive system.
* Motors: Four TT motors for precise movement and control.
* Odometry: Four photo-interruptors to capture encoder data for accurate odometry.
* IMU: MPU9250 providing 9 Degrees of Freedom (DOF) inertial measurement data.
* Lidar: Luna Lidar for environmental scanning and mapping.
* Motor Controller: 
&emsp;- Arduino Uno microcontroller was programmed with Arduino sketch (using PlatformIO) described here [Motor_Driver](https://github.com/JustASimpleCoder/Ardunio_Motor_Driver_Slambot.git)
* STM32 Microcontroller firmware described here [STM32_Drivers](https://github.com/JustASimpleCoder/STM32_Sensors_Slambot), where:<br />
&emsp;- I2C communication with the Lidar and IMU.<br />
&emsp;- IRQ (interrupt requests) for the 4 photo-interuptor encoders.<br />


This repository includes ROS2 packages that integrate these components to enable SLAM functionality.

# Features

* Robot_control node: CLI control with simple wasd control to move robot forward/backward/lefter/righter.

* Robot_gui node: Graphical interface that handles more omnidirectional control.

* Uno_Serial node: handles serial communication of the Arduino Uno Rev3. Microros is too alrge to use with the Arduino Uno, and ROSSERIAl is deprecated in ROS2, so needed to create my own serialization. Check out the correpsonding arduino motor controller here (using PlatofrmIO): [Arduino_Motor_Driver_Slambot](https://github.com/JustASimpleCoder/Ardunio_Motor_Driver_Slambot)

# Visual Representation

![alt text](README_Images/MecanumWheelDiagram.png)

source: https://www.researchgate.net/figure/Movements-of-a-Mecanum-wheel-driven-robot-to-any-directions-side-arrows-indicate-wheel_fig1_367879750

# Future updates
Planned improvements:

* Complete the SLAM, Sensor fusion, etc. 
* Enhancing path planning algorithms for smoother performance.



# useful links
https://www.matec-conferences.org/articles/matecconf/pdf/2021/12/matecconf_mse21_08003.pdf

https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#mobile-robot-kinematics

https://robofoundry.medium.com/ros2-control-differential-drive-robot-project-part-1-mechanical-build-2a323da04992