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
#include <sstream>
#include <iostream>

using std_msgs::Int32;

class EPOS2{

    public:
    int InitCANInterface(const char *ifname);
    void EPOS2Initiate();
    void ControlWordEnabled();
    void CallbackTargetVelocity(const Int32::ConstPtr& TargetVelocity);
    void RosSetting();
    void StopReset();
    void readActualVelocity();
    int HexarrayToInt(unsigned char *buffer, int length);
    template< typename T >
    std::string DecToHexString(T TargetVel);
    std::string StringAppend(std::string a, std::string b);
    unsigned char AscToNibble(char c);
    int HexstringToData(char* arg, unsigned char* data, int MaxDataLength);

    private:

    struct sockaddr_can addr;
    struct can_frame frame;
    struct canfd_frame frame_fd;

    struct iovec iov;
    struct msghdr can_msg;
    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(__u32))];
    struct canfd_frame frame_get;

    int sock_;

    int nbytes;
    struct timeval tv;

    ros::NodeHandle nh_;
    ros::Publisher ActualVelocityPublisher;
    ros::Subscriber TargetVelocitySubscriber;

    double t_prev = ros::Time::now().toSec();

    std::string control_enable = "0F00";
    std::string TargetVel[1];
    std::string TargetVeldata;
    int TargetVel_;

};

int EPOS2::InitCANInterface(const char *ifname)
{    
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(sock == -1){
        printf("Fail to create CAN socket for %s - %m\n",ifname);
        return -1;
    }
    printf("Success to create CAN socket for %s \n",ifname);

    struct  ifreq ifr;
    strcpy(ifr.ifr_name, ifname);
    int ret = ioctl(sock, SIOCGIFINDEX, &ifr);

    if(ret == -1){
        perror("Fail to get CAN interface index -");
        return -1;
    }
    printf("Success to get CAN interface index: %d\n",ifr.ifr_ifindex);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    if(ret == -1)
        perror("Fail to bind CAN socket: -");
        
    return sock;
}

void EPOS2::EPOS2Initiate()
{
    nbytes = 0;
    sock_= InitCANInterface("slcan0");

    std::cout<<"Network Manegerment Initialization \n";

    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x81;
    frame.data[1] = 0x00;

    write(sock_,&frame,sizeof(can_frame));
    sleep(1);

    std::cout<<"Remote Mode\n";
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x01;
    frame.data[1] = 0x00;
    write(sock_,&frame,sizeof(can_frame));
    sleep(1);


    //PDO 
    std::cout<<"Velocity Mode initiate\n";   
    frame.can_id = 0x301;
    frame.can_dlc = 3;
    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    frame.data[2] = 0x03;
    write(sock_,&frame,sizeof(can_frame));

    sleep(1);

    std::cout<<"Controlword shutdown\n";   
    frame.can_id = 0x201;
    frame.can_dlc = 2;
    frame.data[0] = 0x06;
    frame.data[1] = 0x00;
    write(sock_,&frame,sizeof(can_frame));

    sleep(1);
}

void EPOS2::ControlWordEnabled()
{
    std::cout<<"Controlword Enabled\n";   
    frame.can_id = 0x201;
    frame.can_dlc = 2;
    frame.data[0] = 0x0F;
    frame.data[1] = 0x00;
    write(sock_,&frame,sizeof(can_frame));

    sleep(1);

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    //std::cout<<"Motor Velocity Read: \t";
    frame.can_id = 0x381|CAN_RTR_FLAG;
    write(sock_,&frame,sizeof(can_frame));

    iov.iov_base = &frame_get;
    can_msg.msg_name = & addr;
    can_msg.msg_iov = &iov;
    can_msg.msg_iovlen = 1;
    can_msg.msg_control = &ctrlmsg;

    iov.iov_len = sizeof(frame_get);
    can_msg.msg_namelen = sizeof(addr);
    can_msg.msg_controllen = sizeof(ctrlmsg);
    can_msg.msg_flags = 0;
    sleep(1);

    setsockopt(sock_,SOL_SOCKET,SO_RCVTIMEO,(const char*)&tv,sizeof tv);

    nbytes = recvmsg(sock_,&can_msg,0);
    int data = HexarrayToInt(frame_get.data,4);
    //std::cout<<data<<std::endl;
    sleep(1);
}

void EPOS2::CallbackTargetVelocity(const Int32::ConstPtr& TargetVelocity)
{

    TargetVel_ = TargetVelocity->data;
    //std::cout<<"Motor Run\n";   

    TargetVel[0] = DecToHexString(TargetVel_);
    TargetVeldata = StringAppend(control_enable,TargetVel[0]);
    HexstringToData((char *) TargetVeldata.c_str(), frame.data,8);
    
    frame.can_id = 0x401;
    frame.can_dlc = 6;
    /**
    frame.data[0] = 0x0F;
    frame.data[1] = 0x00;
    frame.data[2] = 0x10;
    frame.data[3] = 0x10;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    **/
    write(sock_,&frame,sizeof(can_frame));
    sleep(0.01);
    readActualVelocity();
}

void EPOS2::RosSetting()
{
    TargetVelocitySubscriber = nh_.subscribe("/TargetVel",1,&EPOS2::CallbackTargetVelocity,this);
    ActualVelocityPublisher = nh_.advertise<Int32>("/ActualVel",1);
}

void EPOS2::StopReset()
{
    std::cout<<"Stop Motor \n";
    frame.can_id = 0x401;
    frame.can_dlc = 6;
    frame.data[0] = 0x0F;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    write(sock_,&frame,sizeof(can_frame));

    sleep(2);

    std::cout<<"Reset Remote Mode \n";
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x81;
    frame.data[1] = 0x00;

    write(sock_,&frame,sizeof(can_frame));
    sleep(1);

    std::cout<<"Stop Remote Mode\n";
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x02;
    frame.data[1] = 0x00;
    write(sock_,&frame,sizeof(can_frame));
    sleep(1);

}

void EPOS2::readActualVelocity()
{
    Int32 ActualVel;
    //std::cout<<"Motor Velocity Read: \t";
    frame.can_id = 0x381|CAN_RTR_FLAG;
    write(sock_,&frame,sizeof(can_frame));
 
    nbytes = recvmsg(sock_,&can_msg,0);
    int data = HexarrayToInt(frame_get.data,4);
    //std::cout<<data<<std::endl;
    
    ActualVel.data = data;
    ActualVelocityPublisher.publish(ActualVel);
}

int EPOS2::HexarrayToInt(unsigned char *buffer, int length)
{
    int ActualVel = 0;
    int data = 0;
    for ( int i = 0; i < length; i++)
    {
        ActualVel +=(buffer[i]<<8*i);
        data = (int) buffer[i];
        //printf("Index: %d \t data: %02x \n",i,buffer[i]);
    }
    return ActualVel;
}

template <typename T>
std::string EPOS2::DecToHexString(T TargetVel)
{
    std::stringstream stream;
    std::string s, data;
    stream  <<std::setfill('0')
            <<std::setw(sizeof(T)*2)
            <<std::hex << TargetVel;
    s = stream.str();
    for(int i = 0; i < 4; i++)
        data.append(s.substr(2*(3-i),2));
    //std::cout<<data<<std::endl;
    return data;
}

std::string EPOS2::StringAppend(std::string a,std::string b)
{
    std::string s;
    s.append(a);
    s.append(b);
    return s;
}

unsigned char EPOS2::AscToNibble(char c)
{
    if ((c >= '0') && (c <= '9'))
        return c - '0';

    if ((c >= 'A') && (c <= 'F'))
        return c - 'A' + 10;

    if ((c >= 'a') && (c <= 'f'))
        return c - 'a' + 10;

    return 16;
}

int EPOS2::HexstringToData(char* arg, unsigned char* data, int MaxDataLength)
{
    int length = strlen(arg);
    unsigned char tmp;

    if(!length || length%2 || length > MaxDataLength*2)
        return 1;

    memset(data,0,MaxDataLength);

    for(int i = 0; i < length/2; i++)
    {
        tmp = AscToNibble(*(arg+2*i));
        if (tmp > 0x0F)
            return 1;
        data[i] = (tmp <<4);

        tmp = AscToNibble(*(arg+2*i+1));
        if(tmp > 0x0F)
            return 1;

        data[i] |= tmp;
    }

    return 0;
}