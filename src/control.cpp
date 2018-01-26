/* Author: Bin Xu
   Date: Jan. 22, 2018

   control.cpp
   Description: ServoControl Class for controlling 2 Dynamixel servos
 */

#include <my_dynamixel_tutorial/control.h>

ServoControl::ServoControl(const ros::NodeHandle &nh)
{
    nh.param("servo1_angle", _q1, 1.5);
    nh.param("servo2_angle", _q2, 1.5);
    nh.param("L1", _L1, 0.22360679);
    nh.param("L2", _L2, 0.471699);  // unit is m

    last = 0.0;
    current = 0.0;

    ros::NodeHandle n;
    

    // set up publisher
    _pose_pub = n.advertise<std_msgs::Float64>("/tilt_controller/command", 1);
}

void ServoControl::run(double frequency)
{
    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/frequency), 
        &ServoControl::iteration, this);
    ros::spin();
}

void ServoControl::iteration(const ros::TimerEvent& e)
{
    // do some calculation to generate rotation angle of servos


    // give const value to rotation angle
    std_msgs::Float64 msg;

    // switch the rotation angle based on time interval
    current = ros::Time::now().toSec();
    if (abs(current - last) > 2.0)
    {
        _q1 = -_q1;
        last = current;
        current = ros::Time::now().toSec();
    }

    msg.data = _q1;
    _pose_pub.publish(msg);
}