# dynamixel-tutorial

A simple ROS package to control dynamixel servo using **dynamixel-motor**.

## Prerequisites:
1. [ROS installation](http://wiki.ros.org/indigo/Installation/Ubuntu)
2. [dynamixel-motor installation](http://wiki.ros.org/dynamixel_motor?distro=indigo)

```
sudo apt-get install ros-%ROS_DISTRO%-dynamixel-motor

sudo apt-get install ros-indigo-dynamixel-motor      (ROS Indigo for instance)
```

## Issues:
1. add how to modify the id of Dynamixel Servos, [this](https://github.com/ROBOTIS-GIT/dynamixel-workbench) can help
2. cannot shut down the servo after killing the roslaunch process (servo overheating problem)
