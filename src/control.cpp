/* Author: Bin Xu
   Date: Jan. 22, 2018

   control.cpp
   Description: ServoControl Class for controlling 2 Dynamixel servos with range finder sensor readings
 */

#include <my_dynamixel_tutorial/control.h>

ServoControl::ServoControl(const ros::NodeHandle &nh)
{
    nh.param("servo1_angle", _q1, 0.0);
    nh.param("servo2_angle", _q2, 0.0);
    nh.param("L_MIN", _L_MIN, 0.5);
    nh.param("L_MAX", _L_MAX, 1.5);
    nh.param("l0", _l0, 0.1);
    nh.param("l1", _l1, 0.22360679);
    nh.param("l2", _l2, 0.471699);  // unit is m
    // nh.param("range", _range, 0.0);

    _range = 0.0;    // initialize _range as 0.0
    _alpha = 0.5;

    last = 0.0;
    current = 0.0;

    ros::NodeHandle n;
    
    // set up listener
    // rostopic: "/sf30/range"
    _range_sub = n.subscribe("range", 1000, &ServoControl::rangeCallback, this);

    // set up publisher
    _pose_pub_1 = n.advertise<std_msgs::Float64>("/joint1_controller/command", 1);
    _pose_pub_2 = n.advertise<std_msgs::Float64>("/joint2_controller/command", 1);
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
    double alpha1 = 0.0, alpha2 = 0.0, alpha = 0.0;
    double l = 0.0;
    // do some calculation to generate rotation angle of servos
    // negelect when _range is smaller than the default distance
    if (_range > _L_MIN && _range < _L_MAX)
    {
        // direct distance
        alpha1 = atan2(_range, _l0);
        l = sqrt(_range*_range+_l0*_l0);
        alpha2 = acos((l*l+_l1*_l1-_l2*_l2)/(2*l*_l1));
        alpha = PI - alpha1 - alpha2;
        _q1 = alpha - _alpha;
        _q2 = -(alpha - _alpha);
        
        ROS_INFO("[ROS_INFO] Servo 1: %lf\tServo 2: %lf\n", _q1, _q2);
    }

    // set saturation on rotational angles
    if (_q1 > 2.0)
        _q1 = 2.0;
    if (_q1 < -2.0)
        _q1 = -2.0;
    
    if (_q2 > 2.0)
        _q2 = 2.0;
    if (_q2 < -2.0)
        _q2 = -2.0;


    // give const value to rotation angle
    // in radians
    std_msgs::Float64 msg1;
    std_msgs::Float64 msg2;

    /*
    // switch the rotation angle based on time interval
    current = ros::Time::now().toSec();
    if (abs(current - last) > 2.0)
    {
        _q1 = -_q1;
        _q2 = -_q2;
        last = current;
        current = ros::Time::now().toSec();
    }
    */

    msg1.data = _q1;
    msg2.data = _q2;
    _pose_pub_1.publish(msg1);
    _pose_pub_2.publish(msg2);
}

void ServoControl::rangeCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("[ROS_INFO] Reading range finder message: [%f]", msg->ranges[0]);

    // set the range finder reading _range
    // if two readings are larger than 0.1, then reset the _range
    if (abs(_range - msg->ranges[0]) > 0.1)
        _range = msg->ranges[0];
}