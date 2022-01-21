#include <esprobot/OpenInterface.h>

esprobot::OpenInterface::OpenInterface(const char *new_ip, int port, int _Opcode_length, int _Opcode_Buffer_Size, char *_Opcodes)
{
ip_add_ = new_ip;
port_ = port;
_encoder_ma = 0;
_encoder_mb = 0;
status = 0;
this->setcharSensorPackets(_Opcodes, _Opcode_length, _Opcode_Buffer_Size);
}

esprobot::OpenInterface::~OpenInterface()
{
	this->closeTelnetPort();
}

int esprobot::OpenInterface::openTelnetPort(bool full_control)
{
    //Create socket
    sock = socket(AF_INET , SOCK_STREAM , 0);
    if (sock == -1) {
        //perror("Could not create socket. Error");
	status = -1;
        return -1;
    }
 
    server.sin_addr.s_addr = inet_addr(ip_add_);
    server.sin_family = AF_INET;
    server.sin_port = htons(port_);
 
    //Connect to remote server
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0) {
        //perror("connect failed. Error");
	status = -1;
        return -1;
    }
status = 1;
return 0;
}

int esprobot::OpenInterface::closeTelnetPort()
{
	close(sock);
	return 0;
}

void esprobot::OpenInterface::negotiate(int sock, char *buf, int len) {
    int i;
     

    if (buf[1] == DO && buf[2] == CMD_WINDOW_SIZE) {
        unsigned char tmp1[10] = {255, 251, 31};
        if (send(sock, tmp1, 3 , 0) < 0)
            exit(1);
         
        unsigned char tmp2[10] = {255, 250, 31, 0, 80, 0, 24, 255, 240};
        if (send(sock, tmp2, 9, 0) < 0)
            exit(1);
        return;
    }
     
    for (i = 0; i < len; i++) {
        if (buf[i] == DO)
            buf[i] = WONT;
        else if (buf[i] == WILL)
            buf[i] = DO;
    }
 
    if (send(sock, buf, len , 0) < 0)
        exit(1);
}

int esprobot::OpenInterface::buffer2signed_int(unsigned char * buffer, int index)
{
	int16_t signed_int;
	
	memcpy(&signed_int, buffer+index, 2);
	signed_int = ntohs(signed_int);
	
	return (int)signed_int;
}

int esprobot::OpenInterface::buffer2unsigned_int(unsigned char * buffer, int index)
{
	uint16_t unsigned_int;

	memcpy(&unsigned_int, buffer+index, 2);
	unsigned_int = ntohs(unsigned_int);
	
	return (int)unsigned_int;
}

void esprobot::OpenInterface::calculateOdometry()
{	
	double dist = (_encoder_mb*ESPROB_PULSES_TO_M + _encoder_ma*ESPROB_PULSES_TO_M) / 2.0; 
	double ang = (_encoder_mb*ESPROB_PULSES_TO_M - _encoder_ma*ESPROB_PULSES_TO_M) / ESPROB_AXLE_LENGTH;

	// Update odometry
	this->odometry_yaw_ = NORMALIZE(this->odometry_yaw_ + ang);			// rad
	this->odometry_x_ = this->odometry_x_ + dist*cos(odometry_yaw_);		// m
	this->odometry_y_ = this->odometry_y_ + dist*sin(odometry_yaw_);		// m
}

int esprobot::OpenInterface::setcharSensorPackets(char * new_sensor_packets, int new_num_of_packets, int new_buffer_size)
{
	if(sensor_packets_ == NULL)
	{
		delete [] sensor_packets_;
	}
	
	num_of_packets_ = new_num_of_packets;
	sensor_packets_ = new char[num_of_packets_];
	
	for(int i=0 ; i<num_of_packets_ ; i++)
	{
		sensor_packets_[i] = new_sensor_packets[i];
	}

	stream_defined_ = false;
	packets_size_ = new_buffer_size;
	return(0);
}

// *****************************************************************************
// Read the sensors stream
int esprobot::OpenInterface::streamSensorPackets()
{
	char data_buffer[packets_size_];
	int len=0;

	if(!stream_defined_)
	{
		char cmd_buffer[num_of_packets_+2];
		int index = 0;

		// Fill in the command buffer to send to the robot
		cmd_buffer[index++] = ':';			// Stream
		cmd_buffer[index++] = '#';			// Number of packets
		for(int i=0 ; i<num_of_packets_ ; i++)
		{
			cmd_buffer[index++] = sensor_packets_[i];		// The packet IDs
		}
		if (send(sock, cmd_buffer, index, 0) < 0)
		{
		stream_defined_ = true;
		}
		else
		{
		return -1;
		}
	}
	len = recv(sock , data_buffer, packets_size_, 0);
	
	return this->parseSensorPackets((unsigned char*)data_buffer, len);
}

// *****************************************************************************
// Read the sensors
int esprobot::OpenInterface::getSensorPackets(int timeout)
{
	char cmd_buffer[num_of_packets_+2];
	char data_buffer[packets_size_];
	int index = 0;
	int len = 0;
	// Fill in the command buffer to send to the robot
	cmd_buffer[index++] = ':';			// Stream
	cmd_buffer[index++] = '#';			// Number of packets
	cmd_buffer[index++] = 'D';
	
	for(int i=0 ; i<num_of_packets_ ; i++)
	{
	cmd_buffer[index++] = sensor_packets_[i];		// The packet IDs
	}
	
	int rtn = 0;
	rtn = send(sock, cmd_buffer, index, 0);
	if ( rtn < 0)
	{
//	std::cout<<"TelNet write error";
	return -1;
	}
	stream_defined_ = true;
	
	rtn = read(sock, data_buffer, packets_size_);
	if ( rtn < 0)
	{
//	std::cout<<"TelNet read error";
	return -1;
	}
//	std::cout<<"TelNet read completed";
	
	return this->parseSensorPackets((unsigned char*)data_buffer, len);
//	return 0;
}

// Parse sensor data
int esprobot::OpenInterface::parseSensorPackets(unsigned char * buffer , size_t buffer_length)
{	
	//if(buffer_length != packets_size_)
	//{
		// Error wrong packet size
	//	return(-1);
	//}

	unsigned int index = 2;
	
	while(index < packets_size_)
	{
		if(buffer[index] == 'A')		index += parseControl(buffer, index);
		else if(buffer[index]== 'B')		index += parseMotorDIRData(buffer, index);
		else if(buffer[index]== 'C')		index += parseMotorDuty(buffer, index);
		else if(buffer[index]== 'D')		index += parseEncoderData(buffer, index);	
		else index++;
	}
	return(0);
}

int esprobot::OpenInterface::parseControl(unsigned char * buffer, int index)
{
	_control = buffer[++index];
	return 1;
}

int esprobot::OpenInterface::parseMotorDIRData(unsigned char * buffer, int index)
{
	_dir_mab = buffer[++index];
	return 1;
}

int esprobot::OpenInterface::parseMotorDuty(unsigned char * buffer, int index)
{
	_duty_ma = buffer2unsigned_int(buffer, ++index);
	index += 2;
	_duty_mb = buffer2unsigned_int(buffer, index);
	return 4;
}

int esprobot::OpenInterface::parseEncoderData(unsigned char * buffer, int index)
{
	_encoder_ma = buffer2unsigned_int(buffer, ++index);
	index += 2;
	_encoder_mb = buffer2unsigned_int(buffer, index);
	return 4;
}

int esprobot::OpenInterface::SetMotorData(char control, char dir, int pwma, int pwmb)
{
	char buffer[11];
	int index = 0;
	buffer[index++] =':';
	buffer[index++] ='#';
	buffer[index++] ='E';
	buffer[index++] = control;
	buffer[index++] = dir;
	buffer[index++] = (pwma >> 7) & 0xFF;
	buffer[index++] = pwma & 0xFF;
	buffer[index++] = (pwmb >> 7) & 0xFF;
	buffer[index++] = pwmb & 0xFF;
	index++;

	int rtn = 0;
	rtn = send(sock, buffer, index, 0);	
	
	return rtn;
}

int esprobot::OpenInterface::SetEncoderData(int cnta, int cntb)
{
	char buffer[11];
	int index = 0;
	buffer[index++] =':';
	buffer[index++] ='#';
	buffer[index++] ='F';
	buffer[index++] = (cnta >> 7) & 0xFF;
	buffer[index++] = cnta & 0xFF;
	buffer[index++] = (cntb >> 7) & 0xFF;
	buffer[index++] = cntb & 0xFF;
	index++;
	int rtn = 0;
	rtn = send(sock, buffer, index, 0);	
	
	return rtn;
}

char esprobot::OpenInterface::Get_Motor_Control()
{
	return _control;
}

char esprobot::OpenInterface::Get_Motor_Direction()
{
	return _dir_mab;
}

int esprobot::OpenInterface::Get_Motor_A_Duty()
{
	return _duty_ma;
}

int esprobot::OpenInterface::Get_Motor_B_Duty()
{
	return _duty_mb;
}

int esprobot::OpenInterface::Get_Encoder_A_Count()
{
	return _encoder_ma;
}
int esprobot::OpenInterface::Get_Encoder_B_Count()
{
	return _encoder_mb;
}

