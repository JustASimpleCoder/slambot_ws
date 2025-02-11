
# 4-Wheel Mecanum Drive SLAM Robot

This repository contains ROS2 packages for exploring SLAM (Simultaneous Localization and Mapping) with a 4-wheel differential mecanum drive robot. The robot is built using:

* Chassis: Basic robot chassis with a 4-wheel mecanum drive system.
* Motors: Four TT motors for precise movement and control.
* Odometry: Four photo-interruptors to capture encoder data for accurate odometry.
* IMU: MPU9250 providing 9 Degrees of Freedom (DOF) inertial measurement data.
* Lidar: Luna Lidar for environmental scanning and mapping.
* Arduino Uno was programmed with Arduino sketch (using PlatformIO) described here [Motor_Driver](https://github.com/JustASimpleCoder/Ardunio_Motor_Driver_Slambot.git), where:<br />
&emsp;- Motor Controller that accepts serial communication with basic ASCII characters. 
* STM32 Microcontroller firmware described here [STM32_Drivers](https://github.com/JustASimpleCoder/STM32_Sensors_Slambot), where:<br />
&emsp;- I2C communication with the Lidar and IMU.<br />
&emsp;- IRQ (interrupt requests) for the 4 photo-interuptor encoders.<br />


This repository includes ROS2 packages that integrate these components to enable SLAM functionality.

# Features

* Robot_control node: CLI control with simple omnidirectional control, you can enter: 
enum class RobotMovement: char{
    STOP = 'x',  <br />
    MOVE_FORWARD = 'w',  <br />
    MOVE_BACKWARD = 's', <br />
    MOVE_LEFT = 'l', <br />
    MOVE_RIGHT = 'r', <br />
    ROTATE_LEFT = 'a', <br />
    ROTATE_RIGHT = 'd', <br />
    DIAG_FORWARD_RIGHT = 'e', <br />
    DIAG_BACKWARD_RIGHT = 'c', <br />
    DIAG_FORWARD_LEFT = 'q', <br />
    DIAG_BACKWARD_LEFT = 'z', <br />
    FASTER = '+', <br />
    SLOWER = '-', <br />
    INVALID = '?' <br />
}; <br />
to stop the node you must enter "kill" (instead of usual Ctrl + c). This node also handles serial communication over a given port using the termios.h library where the code is located in /slambot_ws/src/robot_control/src/serial_Communication.cpp. In /slambot_ws/src/robot_control/src/controller_publisher.cpp, we are using a thread to handle waiting for user input, before publishing it over the "motor_control" topic. 

* Robot_gui node: Graphical interface that handles continuous input stream with the omnidirectional control.

* Uno_Serial node: handles serial communication of the Arduino Uno Rev3. Microros is too alrge to use with the Arduino Uno, and ROSSERIAl is deprecated in ROS2, so needed to create my own serialization. Check out the correpsonding arduino motor controller here (using PlatofrmIO): [Arduino_Motor_Driver_Slambot](https://github.com/JustASimpleCoder/Ardunio_Motor_Driver_Slambot)

* Agent_ws: microros agent that is used to communicate with the STM32-F446RE microcontroller, more details are described here : [STM32_Sensors_Slambot](https://github.com/JustASimpleCoder/STM32_Sensors_Slambot)

# how to use

```
cd ~/slambot_ws
```
Source ros2 if you havent alreadying put it into you /.bashrc.
```
source /opt/ros/jazzy/setup.bash
```
Clean previous builds (may not be necessary first time):
```
sudo rm -rf build install log
```
Update any ros dependencies and build the project 
```
rosdep update && rosdep install --from-path src --ignore-src -y
colcon build
```
To launch the manual CLI control, run the following package: 
```
source install/setup.bash
ros2 run robot_control motor_control_node
```

To launch the agent_ws, open another terminal and go to the agent directory: 
```
cd ~/slambot_ws/agent_ws
```
It should already be built if you ran colcon build above, but if you have not then 
```
sudo rm -rf build install log
rosdep update && rosdep install --from-path src --ignore-src -y
colcon build
```

Then build the agent packages: 
```
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
```
Then run the agent (double check what port you are using and the baud rate): 
```
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyACM0
```
If you need more help then you can run: 
```
ros2 run micro_ros_agent micro_ros_agent --help
```


# Visual Representation

![alt text](README_Images/MecanumWheelDiagram.png)

source: https://www.researchgate.net/figure/Movements-of-a-Mecanum-wheel-driven-robot-to-any-directions-side-arrows-indicate-wheel_fig1_367879750

# Future updates
Planned improvements:

* Complete the SLAM, Sensor fusion, etc. 
* Enhancing path planning algorithms for smoother performance.
* Add Launch files to simplify running the robot



# useful links
[Kinematic Model of a Four Mecanum Wheeled Mobile Robot](https://www.matec-conferences.org/articles/matecconf/pdf/2021/12/matecconf_mse21_08003.pdf)

[Kinematics of differential drive](https://www.matec-conferences.org/articles/matecconf/pdf/2021/12/matecconf_mse21_08003.pdf)

[Wheeled Mobile Robot Kinematics fo omnidirectional wheeled mobile robots](https://control.ros.org/jazzy/doc/ros2_controllers/doc/mobile_robot_kinematics.html)

[Medium article on Ros2_Control Differential Drive](https://robofoundry.medium.com/ros2-control-differential-drive-robot-project-part-1-mechanical-build-2a323da04992)

[Trajectory tracking control of a 4wd mecanum](https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/iet-cta.2018.6127#cth2bf01660-bib-0013)