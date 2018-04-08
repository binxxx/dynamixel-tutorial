/* Author: Bin Xu
   Date: Jan. 22, 2018

   control.cpp
   Description: ServoControl Class for controlling 2 Dynamixel servos with range finder sensor readings
 */

#include <my_dynamixel_tutorial/control.h>

ServoControl::ServoControl(const ros::NodeHandle &nh)
{
    nh.param("servo1_angle", this->_q1_default, 0.0);
    nh.param("servo2_angle", this->_q2_default, 0.0);
    nh.param("L_MIN", this->_L_MIN, 0.5);
    nh.param("L_MAX", this->_L_MAX, 0.69);
    nh.param("l0", this->_l0, 0.05);
    nh.param("l1", this->_l1, 0.22360679);
    nh.param("l2", this->_l2, 0.471699);  // unit is m
    // nh.param("range", _range, 0.0);

    this->_q1 = 0.0;
    this->_q2 = 0.0;

    this->_offset = 0.05;
    this->_range = 0.0;   // initialize _range as 0.0
    this->_alpha = atan2(1.,2.);   // default angle offset

    this->_state = false;

    last = 0.0;
    current = 0.0;

    // set actuator saturation
    this->_saturation_max = 0.90; // ( ~ 51 degrees)
    this->_saturation_min = -0.90;

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
    if (this->_state)
    {
        ROS_INFO("inverse kinematics!!!");
        ROS_INFO("range is %f", _range);
        double alpha1 = 0.0, alpha2 = 0.0, alpha = 0.0;
        double l = 0.0;
        double range = this->_range;
        // do calculation to generate rotation angle of servos
        // negelect when _range is smaller than the default distance
        if (range >= _L_MIN && range <= _L_MAX)
        {
            // ROS_INFO("range is %f", _range);
            // direct distance
            alpha1 = atan2(range, _l0);
            l = sqrt(range*range+_l0*_l0);
            alpha2 = acos((l*l+_l1*_l1-_l2*_l2)/(2*l*_l1));
            alpha = PI - alpha1 - alpha2;
            this->_q1 = -(alpha - _alpha);
            this->_q2 = alpha - _alpha;
            
            ROS_INFO("[ROS_INFO] Servo 1: %lf\tServo 2: %lf\n", _q1, _q2);

            // set saturation on rotational angles
            if (this->_q1 > _saturation_max)
                this->_q1 = _saturation_max;
            if (this->_q1 < _saturation_min)
                this->_q1 = _saturation_min;
            
            if (this->_q2 > _saturation_max)
                this->_q2 = _saturation_max;
            if (this->_q2 < _saturation_min)
                this->_q2 = _saturation_min;
        }
        else
        {
            ROS_INFO("[ROS_INFO] distance out of range, current range is %f\n", range);
        }
    }
    else
    {
        // end manipulator control
        // set rotational angle to zero (or negative?)
        // ROS_INFO("[ROS_INFO] reset manipulator!!!");
        // this->_q1 = this->_q2 = 0.0;
        this->_q1 = this->_q1_default;
        this->_q2 = this->_q2_default;
    }

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

    msg1.data = this->_q1;
    msg2.data = this->_q2;
    _pose_pub_1.publish(msg1);
    _pose_pub_2.publish(msg2);
}

void ServoControl::rangeCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // ROS_INFO("[ROS_INFO] Reading range finder message: [%f]", msg->ranges[0]);

    // set the range finder reading _range
    // if two readings are larger than 0.1, then reset the _range
    if (fabs(this->_range - msg->ranges[0]) > 0.01)
    {
        this->_range = msg->ranges[0] - this->_offset;
    }
    // ROS_INFO("range is %f", _range);
}

bool ServoControl::startManipulator(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ROS_INFO("[ROS_INFO] Start manipulator control!!!");

    this->_state = true;
    return true;
}

bool ServoControl::endManipulator(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ROS_INFO("[ROS_INFO] End manipulator control!!!");

    this->_state = false;
    return true;
}