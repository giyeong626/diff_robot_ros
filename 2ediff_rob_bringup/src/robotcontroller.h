#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <sstream>
#include <array>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <string.h>
#include <atomic>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>

class RobotController
{
public:
    void ConnectUsb(const std::string&);
    int Kinematics(double, double, double);
    void ResetCan(void);
    int HubMotorVel(int32_t, float);
    int getOdometry(double*,double*,double*);
    void Reconnection();
    void getRpm(int32_t);
    void HubMotorIdle(int32_t);
    void HubMotorClosedLoop(int32_t);

    int HubMotorId1 = 33;
    int HubMotorId2 = 34;

    double rpm[2];

private:
    uint8_t buf[18] = {0x02, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
    uint8_t data[8] = {0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t reset_cmd[18] = {0x02, 0x80, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0x03};
    uint8_t reset_cmd2[18] = {0x02, 0x80, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x89, 0x03};
    uint8_t readBuf1[18];
    uint8_t readBuf2[18];
    uint8_t readBuf3[18];
    uint8_t readBuf4[18];
    uint8_t readBuf5[18];

    uint16_t maxSpeed = 100;
    
    int m_socketID;
    
    double pi = 3.1415926535;
    double WheelRadius = 0.0635; //radius[m]

    double L1 = 0.350; //Width[m]

    double rpm1;
    double rpm2;

    int current_feed = 0;
    int Cmd_Id_Vel = 0x00D;
    int Cmd_Id_Axis_Requested_State = 0x007;
    int Cmd_Id_Encoder = 0x015;

    sockaddr_in m_socketDescriptor;

    std::string m_receiveBuffer;

    uint32_t m_dataPointCount;

    tcflag_t GetBaudrateFlag(uint32_t baudrate);

    bool port_opened_;
    std::atomic_bool is_running_;
    std::string device_port_;
    uint32_t baudrate_;

};
