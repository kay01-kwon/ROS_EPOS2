#include <epos2_test/epos2_control.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"epos2_control");

    EPOS2CTRL epos2_control;

    while (ros::ok())
    {
        ros::spinOnce();
    }
}