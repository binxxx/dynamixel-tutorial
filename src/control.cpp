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
    nh.param("L_MAX", _L_MAX, 0.695);
    nh.param("l0", _l0, 0.05);
    nh.param("l1", _l1, 0.22360679);
    nh.param("l2", _l2, 0.471699);  // unit is m
    // nh.param("range", _range, 0.0);

    _range = 0.0;   // initialize _range as 0.0
    _alpha = atan2(1.,2.);   // default angle offset

    _state = false;

    last = 0.0;
    current = 0.0;

    // set actuator saturation
    _saturation_max = 2.0;
    _saturation_min = -2.0;

    ros::NodeHandle n;
    
    // set up listener
    // rostopic: "/sf30/range"
    _range_sub = n.subscribe("/sf30/range", 1000, &ServoControl::rangeCallback, this);

    // set up publisher
    _pose_pub_1 = n.advertise<std_msgs::Float64>("/joint1_controller/command", 1);
    _pose_pub_2 = n.advertise<std_msgs::Float64>("/joint2_controller/command", 1);

    // set up service
    _service_start_manipulator = n.advertiseService("start_manipulator",
                                                    &ServoControl::startManipulator, this);
    _service_end_manipulator = n.advertiseService("end_manipulator",
                                                    &ServoControl::endManipulator, this);
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
    if (_state)
    {
        ROS_INFO("inverse kinematics!!!");
        ROS_INFO("range is %f", _range);
        double alpha1 = 0.0, alpha2 = 0.0, alpha = 0.0;
        double l = 0.0;
        // do some calculation to generate rotation angle of servos
        // negelect when _range is smaller than the default distance
        if (_range > _L_MIN && _range < _L_MAX)
        {
            // ROS_INFO("range is %f", _range);
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
        if (_q1 > _saturation_max)
            _q1 = _saturation_max;
        if (_q1 < _saturation_min)
            _q1 = _saturation_min;
        
        if (_q2 > _saturation_max)
            _q2 = _saturation_max;
        if (_q2 < _saturation_min)
            _q2 = _saturation_min;
    }
    else
    {
        // end manipulator control
        // set rotational angle to zero (or negative?)
        _q1 = _q2 = 0.0;
    }

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
    // ROS_INFO("[ROS_INFO] Reading range finder message: [%f]", msg->ranges[0]);

    // set the range finder reading _range
    // if two readings are larger than 0.1, then reset the _range
    if (fabs(_range - msg->ranges[0]) > 0.01)
    {
        _range = msg->ranges[0];
    }
    // ROS_INFO("range is %f", _range);
}

bool ServoControl::startManipulator(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ROS_INFO("[ROS_INFO] Start manipulator control!!!");

    _state = true;
    return true;
}

bool ServoControl::endManipulator(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ROS_INFO("[ROS_INFO] End manipulator control!!!");

    _state = false;
    return true;
}