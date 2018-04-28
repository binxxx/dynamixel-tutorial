/* Author: Bin Xu
   Date: Jan. 22, 2018
   
   control.h
   Description: ServoControl Class for controlling 2 Dynamixel servos
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>

#include <dynamixel_msgs/JointState.h>

#include <math.h>

#define PI 3.14159265

class ServoControl
{
public:
    ServoControl(const ros::NodeHandle &nh);
    ~ServoControl() {}

    void run(double frequency);


private:
    double _q1; // servo 1 rotation angle
    double _q2; // servo 2 rotation angle
    double _L_MIN, _L_MAX;
    double _l0, _l1, _l2;
    double _q1_default;
    double _q2_default;

    float _offset; // add offset due to mechanism

    float _range;
    double _alpha;

    double _load;
    double _force;
    double _force_ref;

    bool _state;

    double last;
    double current;

    double _saturation_max;
    double _saturation_min;

    bool _is_force_control_flag;

    double _K;  // force controller proportional gain

    ros::Subscriber _range_sub;
    ros::Subscriber _joint_state_sub;

    ros::Publisher _pose_pub_1;
    ros::Publisher _pose_pub_2;

    // ros service
    ros::ServiceServer _service_start_manipulator;
    ros::ServiceServer _service_end_manipulator;
    ros::ServiceServer _service_start_force_control;
    ros::ServiceServer _service_end_force_control;

    void iteration(const ros::TimerEvent& e);
    void rangeCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void stateCallback(const dynamixel_msgs::JointState::ConstPtr& msg);
    bool startManipulator(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool endManipulator(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool startForceControl(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool endForceControl(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
};

#endif