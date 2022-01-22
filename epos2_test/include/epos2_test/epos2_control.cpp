#include "epos2_control.h"

// Constructor
EPOS2CTRL::EPOS2CTRL()
{
    TargetVelocitySubscriber = nh_.subscribe("/TargetVel",1,&EPOS2CTRL::CallbackTargetVelocity,this);
    ActualVelocityPublisher = nh_.advertise<Int32>("/ActualVel",1);
    ActualPositionPublisher = nh_.advertise<Int32>("/ActualPos",1);

    sock_ = InitiateCANInterface("slcan0");


    iov.iov_base = &frame_get;
    can_msg.msg_name = & addr;
    can_msg.msg_iov = &iov;
    can_msg.msg_iovlen = 1;
    can_msg.msg_control = &ctrlmsg;

    iov.iov_len = sizeof(frame_get);
    can_msg.msg_namelen = sizeof(addr);
    can_msg.msg_controllen = sizeof(ctrlmsg);
    can_msg.msg_flags = 0;

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    std::cout<<"Network Manegement Node Initialization \n";

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

}

int EPOS2CTRL::InitiateCANInterface(const char *ifname)
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

void EPOS2CTRL::VelocityModeSet()
{
    std::cout<<"Velocity Mode Setup\n";   
    frame.can_id = 0x301;
    frame.can_dlc = 3;
    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    frame.data[2] = (unsigned)VelocityProfileMode;
    write(sock_,&frame,sizeof(can_frame));

    sleep(1);
}

void EPOS2CTRL::PositionModeSet()
{
    std::cout<<"Position Mode Setup\n";
    frame.can_id = 0x301;
    frame.can_dlc = 3;
    frame.data[0] = (unsigned)zero_byte;
    frame.data[1] = (unsigned)zero_byte;
    frame.data[0] = (unsigned)PositionProfileMode;
    write(sock_,&frame,sizeof(can_frame));

    sleep(1);
}

void EPOS2CTRL::ControlwordShutdown()
{
    std::cout<<"Controlword shutdown\n";   
    frame.can_id = 0x201;
    frame.can_dlc = 2;
    frame.data[0] = 0x06;
    frame.data[1] = 0x00;
    write(sock_,&frame,sizeof(can_frame));

    sleep(1);
}

void EPOS2CTRL::CallbackTargetVelocity(const Int32& TargetVelocity)
{

    TargetVel = TargetVelocity.data;

    frame.can_id = 0x401;
    frame.can_dlc = 6;
    frame.data[0] = 0x0F;
    frame.data[1] = 0x00;

    for (int i = 0; i < 4; i++)
    {
        frame.data[i+2] = unsigned((TargetVel >> 8*i));
    }

    write(sock_,&frame,sizeof(can_frame));
    
    PositionRequest();
    VelocityRequest();

    ReadActualVelocity();
    ReadActualPosition();
}

void EPOS2CTRL::ControlWordEnable()
{
    std::cout<<"Controlword Enabled\n";   
    frame.can_id = 0x201;
    frame.can_dlc = 2;
    frame.data[0] = 0x0F;
    frame.data[1] = 0x00;
    write(sock_,&frame,sizeof(can_frame));


    setsockopt(sock_,SOL_SOCKET,SO_RCVTIMEO,(const char*)&tv,sizeof tv);
    sleep(1);
}

void EPOS2CTRL::PositionRequest()
{
    frame.can_id = 0x281|CAN_RTR_FLAG;
    write(sock_,&frame,sizeof(can_frame));
}

void EPOS2CTRL::VelocityRequest()
{
    frame.can_id = 0x381|CAN_RTR_FLAG;
    write(sock_,&frame,sizeof(can_frame));    
}

void EPOS2CTRL::ReadActualPosition()
{
    recvmsg(sock_,&can_msg,0);

    if(frame_get.can_id == 0x281)
        ActualPos.data = ((frame_get.data[3]<<24)|(frame_get.data[2]<<16)|(frame_get.data[1]<<8)|(frame_get.data[0]));
    
    ActualPositionPublisher.publish(ActualPos);    
}

void EPOS2CTRL::ReadActualVelocity()
{
    recvmsg(sock_,&can_msg,0);
    
    if(frame_get.can_id == 0x381)
        ActualVel.data = ((frame_get.data[3]<<24)|(frame_get.data[2]<<16)|(frame_get.data[1]<<8)|(frame_get.data[0]));
    
    ActualVelocityPublisher.publish(ActualVel);
}


void EPOS2CTRL::StopReset()
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
    
    std::cout<<"Stop Remote Mode\n";
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x02;
    frame.data[1] = 0x00;
    write(sock_,&frame,sizeof(can_frame));
    sleep(1);

}

EPOS2CTRL::~EPOS2CTRL()
{
    StopReset();

    ActualVelocityPublisher.~Publisher();
    TargetVelocitySubscriber.~Subscriber();
}
