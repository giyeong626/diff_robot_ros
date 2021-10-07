#include "robotcontroller.h"

void RobotController::ConnectUsb(const std::string& ip)
{
    
	int serial_port = open(ip.c_str(), O_RDWR | O_NOCTTY);
	if (serial_port == -1){
        std::cout<<"error connecting USB port" << std::endl;
    }
	struct termios port_settings;

	port_settings.c_cflag = B921600; // Set baudrate
	port_settings.c_cflag |= CS8; // 8 bits per byte
	port_settings.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

	port_settings.c_iflag = IGNPAR; // Ignore bytes with parity errors
	port_settings.c_oflag = 0; // Enable raw data output.

	if (tcflush(serial_port, TCIFLUSH) != 0)
	{
		std::cout << "Error while flushing serial port buffers"<<std::endl;
		return;
	}

	if (tcsetattr(serial_port, TCSANOW, &port_settings) != 0)
	{
		std::cout << "Error while setting port attribute"<<std::endl;
		return;
	}
    std::cout << "Motor port connected" << std::endl;

	m_socketID = serial_port;
	port_opened_ = true;

}


void RobotController::ResetCan(void)
{
    write(m_socketID, reset_cmd, 18);
}

int RobotController::HubMotorVel(int32_t ID, float vel)
{
    int32_t ID_send = (ID << 5) + Cmd_Id_Vel;
    uint8_t* vel_b = (uint8_t*) &vel;
    int res = 0;

    buf[4] = ID_send & 0xFF;
    buf[5] = (ID_send >> 8) & 0xFF;
    buf[6] = (ID_send >> 16) & 0xFF;
    buf[7] = (ID_send >> 24) & 0xFF;
    buf[8] = vel_b[0];
    buf[9] = vel_b[1];
    buf[10] = vel_b[2];
    buf[11] = vel_b[3];
    buf[12] = 0x00;
    buf[13] = 0x00;
    buf[14] = 0x00;
    buf[15] = 0x00;
    for (int i = 1; i < 16; i++ )
    {
        buf[16] += buf[i];
    }
    
    if (write(m_socketID, buf, 18) == 18){
        return 0;
    }
    else{
        return 1;
    }
}

void RobotController::HubMotorIdle(int32_t MotorID)
{
    int32_t ID_send = (MotorID << 5) + Cmd_Id_Axis_Requested_State;
    uint8_t Idle = 1;

    buf[4] = ID_send & 0xFF;
    buf[5] = (ID_send >> 8) & 0xFF;
    buf[6] = (ID_send >> 16) & 0xFF;
    buf[7] = (ID_send >> 24) & 0xFF;
    buf[8] = Idle;
    buf[9] = 0x00;
    buf[10] = 0x00;
    buf[11] = 0x00;
    buf[12] = 0x00;
    buf[13] = 0x00;
    buf[14] = 0x00;
    buf[15] = 0x00;
    for (int i = 1; i < 16; i++ )
    {
        buf[16] += buf[i];
    }
    write(m_socketID, buf, 18);
}


void RobotController::HubMotorClosedLoop(int32_t MotorID)
{
    int32_t ID_send = (MotorID << 5) + Cmd_Id_Axis_Requested_State;
    uint8_t ClosedLoop = 8;

    buf[4] = ID_send & 0xFF;
    buf[5] = (ID_send >> 8) & 0xFF;
    buf[6] = (ID_send >> 16) & 0xFF;
    buf[7] = (ID_send >> 24) & 0xFF;
    buf[8] = ClosedLoop;
    buf[9] = 0x00;
    buf[10] = 0x00;
    buf[11] = 0x00;
    buf[12] = 0x00;
    buf[13] = 0x00;
    buf[14] = 0x00;
    buf[15] = 0x00;
    for (int i = 1; i < 16; i++ )
    {
        buf[16] += buf[i];
    }
    write(m_socketID, buf, 18);
}

int RobotController::Kinematics(double Vx, double Vy, double Vomega)
{
    int M1,M2;
    
    double R;

    if(Vomega == 0)
    {
        rpm1 = rpm2 = Vx * 60 / (WheelRadius * 2 * pi);
    }
    else
    {
        R = fabs(Vx) / (-1)*Vomega;
        rpm1 = (-1)*Vomega * copysign(1.0, Vx) * (R + L1 * 0.5) * 60 / (WheelRadius * 2 * pi); // remove 60 for turn/sec
        rpm2 = (-1)*Vomega * copysign(1.0, Vx) * (R - L1 * 0.5) * 60 / (WheelRadius * 2 * pi); // *60 : rpm
    }
    M1 = HubMotorVel(HubMotorId1, -1* rpm1/60);
    M2 = HubMotorVel(HubMotorId2, rpm2/60);

    return 0;
}

int RobotController::getOdometry(double *vx, double *vy, double *vth)
{
    int status = 0;
    uint32_t ID_send = (HubMotorId1 << 5) + 0x009;
    buf[3] = 0x20;
    buf[4] = ID_send & 0xFF;
    buf[5] = (ID_send >> 8) & 0xFF;
    buf[6] = (ID_send >> 16) & 0xFF;
    buf[7] = (ID_send >> 24) & 0xFF;
    buf[8] = 0x00;
    buf[9] = 0x00;
    buf[10] = 0x00;
    buf[11] = 0x00;
    buf[12] = 0x00;
    buf[13] = 0x00;
    buf[14] = 0x00;
    buf[15] = 0x00;
    for (int i = 1; i < 16; i++ )
    {
        buf[16] += buf[i];
    }
    
    write(m_socketID, buf, 18);
    if(read(m_socketID, readBuf4, 18) != 18){
        status += 1;
    }
    float_t output;
    *((uint8_t *)(&output) + 0) = readBuf4[12];
    *((uint8_t *)(&output) + 1) = readBuf4[13];
    *((uint8_t *)(&output) + 2) = readBuf4[14];
    *((uint8_t *)(&output) + 3) = readBuf4[15];
    rpm[0] = output;

    ID_send = (HubMotorId2 << 5) + 0x009;
    buf[3] = 0x20;
    buf[4] = ID_send & 0xFF;
    buf[5] = (ID_send >> 8) & 0xFF;
    buf[6] = (ID_send >> 16) & 0xFF;
    buf[7] = (ID_send >> 24) & 0xFF;
    for (int i = 1; i < 16; i++ )
    {
        buf[16] += buf[i];
    }
    
    write(m_socketID, buf, 18);
    if(read(m_socketID, readBuf5, 18) != 18){
        status += 1;
    }    

    *((uint8_t *)(&output) + 0) = readBuf5[12];
    *((uint8_t *)(&output) + 1) = readBuf5[13];
    *((uint8_t *)(&output) + 2) = readBuf5[14];
    *((uint8_t *)(&output) + 3) = readBuf5[15];
    rpm[1] = output * -1;
    buf[3]= 0x00;

    *vx = -1* (rpm[0] + rpm[1]) * WheelRadius * pi;
    *vth = -1* (rpm[1] - rpm[0]) * WheelRadius * pi / L1;
    *vy = 0.0;

    return status;

}
