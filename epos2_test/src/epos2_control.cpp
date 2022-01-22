#include <epos2_test/epos2_control.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"epos2_control");

    EPOS2CTRL epos2_control;

    epos2_control.VelocityModeSet();
    epos2_control.ControlwordShutdown();
    epos2_control.ControlWordEnable();

    while (ros::ok())
    {
        ros::spinOnce();
    }
}