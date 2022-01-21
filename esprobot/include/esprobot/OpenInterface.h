#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <termios.h>
#include <fcntl.h>
#include <string>
#include <iostream>
#include <math.h> 

#define NODE_VERSION 1.0

#define DO 0xfd
#define WONT 0xfc
#define WILL 0xfb
#define DONT 0xfe
#define CMD 0xff
#define CMD_ECHO 1
#define CMD_WINDOW_SIZE 31

//  EYANTRA Dimensions
#define ESPROB_BUMPER_X_OFFSET		0.001
#define ESPROB_DIAMETER		0.060
#define ESPROB_AXLE_LENGTH		0.120

#define  ESPROB_MAX_LIN_VEL_MM_S	100
#define  ESPROB_MAX_ANG_VEL_RAD_S	1  
#define  ESPROB_MAX_RADIUS_MM		1000

//!  EYANTRA max encoder counts
#define  ESPROB_MAX_ENCODER_COUNTS	20
//!  EYANTRA encoder pulses to meter constant
#define  ESPROB_PULSES_TO_M		0.0094			//(2 * PI * Radius of wheel)/Encoder counts per revolution


#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif

namespace esprobot
{

	class OpenInterface
	{
	public:
	//! Constructor
	/*!
	 * By default the constructor will set the  EYANTRA to read only the encoder counts (for odometry).
	 *
	 *  \param new_serial_port    Name of the serial port to open.
	 *
	 *  \sa setSensorPackets()
	 */
	//OpenInterface(const char * new_serial_port);
	OpenInterface(const char *new_ip, int port, int _Opcode_length, int _Opcode_Buffer_Size, char *_Opcodes);
	//! Destructor
	~OpenInterface();

	//! Open the Telnet port
	int openTelnetPort(bool full_control);
	//! Close the Telnet port
	int closeTelnetPort();

	void negotiate(int sock, char *buf, int len);

//	int setSensorPackets(OI_Packet_ID * new_sensor_packets, int new_num_of_packets, size_t new_buffer_size);

	int getSensorPackets(int timeout);

	//! Stream sensor packets. NOT TESTED
	int streamSensorPackets();
	//! Start stream. NOT TESTED
	int startStream();
	//! Stom stream. NOT TESTED
	int stopStream();

	//! Drive PWM
	/*!
	*  Set the motors pwms. IMPLEMENTED
	*
	*  \param left_pwm  	Left wheel motor pwm.
	*  \param right_pwm  	Right wheel motor pwm.
	*
	*  \return 0 if ok, -1 otherwise.
	*/
	int drivePWM(int left_pwm, int right_pwm);
	
	void resetOdometry();
	void setOdometry(double new_x, double new_y, double new_yaw);

	//!  odometry x
	double odometry_x_;
	//!  odometry y
	double odometry_y_;
	//!  odometry yaw
	double odometry_yaw_;

	void calculateOdometry();
	int SetMotorData(char control, char dir, int pwma, int pwmb);
	int SetEncoderData(int cnta, int cntb);
	char Get_Motor_Control();
	char Get_Motor_Direction();
	int Get_Motor_A_Duty();
	int Get_Motor_B_Duty();
	int Get_Encoder_A_Count();
	int Get_Encoder_B_Count();

	private:
//	unsigned char buf[BUFLEN + 1];
	//! Telnet port to which the robot is connected
	const char *ip_add_;
	int port_;
	int status;
	int sock;
	struct sockaddr_in server;
	//! Stream variable. NOT TESTED
	bool stream_defined_;
	//! Number of packets
	int num_of_packets_;
	//! Array of packets
	char * sensor_packets_;
	//! Total size of packets
	int packets_size_;

	int buffer2signed_int(unsigned char * buffer, int index);
	int buffer2unsigned_int(unsigned char * buffer, int index);

	//! setchar sensor packets. NOT TESTED
	int setcharSensorPackets(char * new_sensor_packets, int new_num_of_packets, int new_buffer_size);

	char _control;
	int _duty_ma;
	int _duty_mb;
	char _dir_mab;

	int _encoder_ma;
	int _encoder_mb;

	//! Parse data
	/*!
	*  Data parsing function. Parses data comming from the  EYANTRA.
	*
	*  \param buffer  			Data to be parsed.
	*  \param buffer_length  	Size of the data buffer.
	*
	*  \return 0 if ok, -1 otherwise.
	*/
	int parseSensorPackets(unsigned char * buffer, size_t buffer_length);
	int parseControl(unsigned char * buffer, int index);
	int parseMotorDIRData(unsigned char * buffer, int index);
	int parseMotorDuty(unsigned char * buffer, int index);
	int parseEncoderData(unsigned char * buffer, int index);
	};

}
