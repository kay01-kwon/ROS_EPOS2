#ifndef EPOS2CONTROL_H
#define EPOS2CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <iostream>
#include <epos2_test/epos2_definition.h>

using std_msgs::Int32;

class EPOS2CTRL{

    public:

    // Constructor : Initiate
    EPOS2CTRL();

    // Initiate CAN interface
    int InitiateCANInterface(const char *ifname);

    // Velocity Profile Mode Set
    void VelocityModeSet();

    // Position Profile Mode Set
    void PositionModeSet();

    void ControlwordShutdown();

    // Callback Function
    void CallbackTargetVelocity(const Int32 & TargetVel_msg);

    // Control word Enable
    void ControlWordEnable();

    // Request Position Value
    void PositionRequest();

    // Request Velocity Value
    void VelocityRequest();

    // Read Position Data
    void ReadActualPosition();

    // Read RPM Data
    void ReadActualVelocity();

    // Stop and Reset
    void StopReset();

    // Destructor
    ~EPOS2CTRL();
    private:

    struct sockaddr_can addr;
    struct can_frame frame;
    struct can_frame frame2;
    struct canfd_frame frame_fd;

    struct iovec iov;
    struct msghdr can_msg;
    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(__u32))];
    struct canfd_frame frame_get;
    struct canfd_frame frame_get2;

    int nbytes;
    int sock_;

    struct timeval tv;

    ros::NodeHandle nh_;
    ros::Publisher ActualVelocityPublisher;
    ros::Publisher ActualPositionPublisher;
    ros::Subscriber TargetVelocitySubscriber;

    double last_time = ros::Time::now().toSec();

    int TargetVel;

    Int32 ActualVel;
    Int32 ActualPos;
};

#endif