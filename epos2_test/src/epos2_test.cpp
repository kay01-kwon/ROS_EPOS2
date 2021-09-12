#include <epos2_test/epos2_test.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "epos2");
    
    EPOS2 epos2_control;

    epos2_control.EPOS2Initiate();
    epos2_control.ControlWordEnabled();

    epos2_control.RosSetting();

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
    }

    epos2_control.StopReset();
}
