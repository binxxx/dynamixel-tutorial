#include <my_dynamixel_tutorial/control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_control_node");
    ros::NodeHandle n("~");
    double frequency;
    n.param("frequency", frequency, 50.0);

    ServoControl servos(n);
    servos.run(frequency);

    // ros::spin();

    return 0;
}