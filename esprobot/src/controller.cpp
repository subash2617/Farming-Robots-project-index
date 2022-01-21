#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <esprobot/OpenInterface.h>
#include <esprobot/espmotor.h>
#include <std_srvs/Empty.h>


std::string ip;
int port;
esprobot::OpenInterface *esprobo;
ros::Time vel_cmd_time;


bool srvClearOdomReceived(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
int rtn = 0;
	rtn = esprobo->SetEncoderData(0, 0) ;

	if(rtn>0) 	ROS_INFO("Encoder Clear Message send sucessfully");
	else	 	ROS_ERROR("Error sending Motor Message");
	return true;
}

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	//esprobo->drive(cmd_vel->linear.x, cmd_vel->angular.z);
}

void cmdMotorReceived(const esprobot::espmotor::ConstPtr& cmd_motor)
{
int rtn = 0;
	vel_cmd_time = ros::Time::now();
	rtn = esprobo->SetMotorData(cmd_motor->control, cmd_motor->dir, cmd_motor->pwma, cmd_motor->pwmb) ;

	if(rtn>0) 	ROS_INFO("Motor Message send sucessfully");
	else	 	ROS_ERROR("Error sending Motor Message");
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "esp_light_node");

	ROS_INFO("esp robot for ROS %.2f", NODE_VERSION);
	
	double last_x, last_y, last_yaw;
	double vel_x, vel_y, vel_yaw;
	double dt;
	int cmd_timeout;

	ros::NodeHandle n;

	XmlRpc::XmlRpcValue Opcode_List;
	int Opcode_Length;
	int Opcode_Buffer_Size;

	n.getParam("/esprobot_node/Opcode/list", Opcode_List);
	n.getParam("/esprobot_node/Opcode/length", Opcode_Length);
	n.getParam("/esprobot_node/Opcode/buffer_size", Opcode_Buffer_Size);

	char Opcodes[Opcode_Length];

	for (int i=0; i<Opcode_Length; i++)
	{
		Opcodes[i] = static_cast<int>(Opcode_List[i]);
		ROS_INFO("OPCode %i",(int)Opcodes[i]);
	}

//end of modification
	
	n.param<std::string>("/esprobot_node/esprobot/ip", ip, "127.0.0.1");
	n.param<int>("/esprobot_node/esprobot/port", port, 23);
	n.param<int>("/esprobot_node/esprobot/cmdtimeout", cmd_timeout, 2);
	esprobo = new esprobot::OpenInterface(ip.c_str(), port, Opcode_Length, Opcode_Buffer_Size, Opcodes);
	
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	tf::TransformBroadcaster odom_broadcaster;
	ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelReceived);
	ros::Subscriber cmd_motor_sub  = n.subscribe<esprobot::espmotor>("/cmd_motor", 1, cmdMotorReceived);
	ros::ServiceServer srv_ClearOdom = n.advertiseService("/clearodom", srvClearOdomReceived);

	if( esprobo->openTelnetPort(true) == 0) ROS_INFO("Connected to esp8266 base robot.");
	else
	{
		ROS_FATAL("Could not connect to esprobo base robot.");
		ROS_BREAK();
	}

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	vel_cmd_time = ros::Time::now();
	//int heartvalue = 0;
	//bool inc = true;
	
	ros::Rate r(10.0);

	while(n.ok())
	{
		current_time = ros::Time::now();
		
		last_x = esprobo->odometry_x_;
		last_y = esprobo->odometry_y_;
		last_yaw = esprobo->odometry_yaw_;
		
		if( esprobo->getSensorPackets(100) == -1) ROS_ERROR("Could not retrieve sensor packets.");
		else esprobo->calculateOdometry();
		
		ROS_INFO("C:%i D:%i DA:%i DB:%i EA:%i EB:%i",esprobo->Get_Motor_Control(),esprobo->Get_Motor_Direction(),esprobo->Get_Motor_A_Duty(),esprobo->Get_Motor_B_Duty(),esprobo->Get_Encoder_A_Count(),esprobo->Get_Encoder_B_Count());
		
		dt = (current_time - last_time).toSec();
		vel_x = (esprobo->odometry_x_ - last_x)/dt;
		vel_y = (esprobo->odometry_y_ - last_y)/dt;
		vel_yaw = (esprobo->odometry_yaw_ - last_yaw)/dt;
		
		int cmd_dt = (current_time - vel_cmd_time).toSec();
		
		if (cmd_dt > cmd_timeout)
		{
		if(esprobo->Get_Motor_Control() != 0 )
		{
		esprobo->SetMotorData(0,0,0,0);
		ROS_WARN("COMMAND VELOCITY TIMEOUT");
		 }
		}
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(esprobo->odometry_yaw_);
		
		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		
		odom_trans.transform.translation.x = esprobo->odometry_x_;
		odom_trans.transform.translation.y = esprobo->odometry_y_;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		
		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		
		//set the position
		odom.pose.pose.position.x = esprobo->odometry_x_;
		odom.pose.pose.position.y = esprobo->odometry_y_;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vel_x;
		odom.twist.twist.linear.y = vel_y;
		odom.twist.twist.angular.z = vel_yaw;
		
		//publish the message
		odom_pub.publish(odom);

		ros::spinOnce();
		r.sleep();
	}
	
//	esprobo->powerDown();
	esprobo->closeTelnetPort();
}
