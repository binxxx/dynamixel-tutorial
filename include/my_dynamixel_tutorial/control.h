/* Author: Bin Xu
   Date: Jan. 22, 2018
   
   control.h
   Description: ServoControl Class for controlling 2 Dynamixel servos
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>

class ServoControl
{
public:
    ServoControl(const ros::NodeHandle &nh);
    ~ServoControl() {}

    void run(double frequency);


private:
    double _q1; // servo 1 rotation angle
    double _q2; // servo 2 rotation angle
    double _L1;
    double _L2;


    ros::Publisher _pose_pub;

    void iteration(const ros::TimerEvent& e);
};

#endif