#include <vector>
#include <string>
#include "mrobot.h"

namespace mrobot
{

const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};
const int SPEED_INFO = 0xaaaa; 
const int GET_SPEED  = 0x5aa5;
const double ROBOT_RADIUS = 105.00;
const double ROBOT_LENGTH = 210.50;

boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/seeker");

boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
boost::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0, 
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};

union sendData
{
	int d;
	unsigned char data[4];
}leftdata, rightdata;


union sendDataINT16
{
	short d;
	unsigned char data[2];
}leftdataINT16, rightdataINT16,yawdataINT16;



union checkSum
{
	short d;
	unsigned char data[1];
}SendCheckSum, ReceiveCheckSum;

union receiveHeader
{
	int d;
	unsigned char data[2];
}receive_command, receive_header;

union sendCommand
{
	int d;
	unsigned char data[2];
}send_command;

union odometry
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}vel_left, vel_right;


MRobot::MRobot():
    x_(0.0), y_(0.0), th_(0.0),
    vx_(0.0), vy_(0.0), vth_(0.0)
{
}

MRobot::~MRobot()
{
}

bool MRobot::init()
{
    // 串口连接
	sp.set_option(boost::asio::serial_port::baud_rate(115200));
	sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	sp.set_option(boost::asio::serial_port::character_size(8));
    
    ros::Time::init();
	current_time_ = ros::Time::now();
	last_time_ = ros::Time::now();
	
    //定义发布消息的名称
    pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);		
    
    return true;
}


unsigned char Get_Sum(unsigned char *ptr, unsigned int len)
{
	unsigned char Sum;
	unsigned char i;
	Sum = 0;
	for (i = 0; i < len; i++)
	{
		Sum = Sum + *(ptr + i);
	}
	return Sum;
}


bool MRobot::readSpeed(double RobotV_y)
{
	/*int i, length = 0;
	unsigned char checkSum;
    	unsigned char buf[16];
    
    // 读取串口数据
	boost::asio::read(sp, boost::asio::buffer(buf));
	ros::Time curr_time;
	for (i = 0; i < 2; i++)
		receive_header.data[i] = buf[i];
	
    // 检查信息头
	if (receive_header.data[0] != header[0] || receive_header.data[1] != header[1])
	{
		ROS_ERROR("Received message header error!");
        	return false;
	}

	for (i = 0; i < 2; i++)
		receive_command.data[i] = buf[i + 2];
	
	length = buf[4];
	checkSum = getCrc8(buf, 5 + length);
    
    // 检查信息类型
	if(receive_command.d != SPEED_INFO)
	{
		ROS_ERROR("Received command error!");
        	return false;
	}

    // 检查信息尾
	if (buf[6 + length] != ender[0] || buf[6 + length + 1] != ender[1])
	{
		ROS_ERROR("Received message ender error!");
        	return false;
	}
    
    // 检查信息校验值
	ReceiveCheckSum.data[0] = buf[5 + length];
	if (checkSum != ReceiveCheckSum.d)
	{
		ROS_ERROR("Received data check sum error!");
        	return false;
	}

    // 读取速度值
	for(i = 0; i < 4; i++)
	{
		leftdata.data[i]  = buf[i + 5];
		rightdata.data[i] = buf[i + 9];
	}*/


	int i, length = 0;
	unsigned char checkSum;
    unsigned char buf[12];
    
    // 读取串口数据
	boost::asio::read(sp, boost::asio::buffer(buf));
	ros::Time curr_time;
	for (i = 0; i < 2; i++)
		receive_header.data[i] = buf[i];
	//std:: cout<<receive_header.data[0]<<std::endl;
	//std:: cout<<receive_header.data[1]<<std::endl;
	//ROS_ERROR("reading");
    // 检查信息头
	if (receive_header.data[0] != 0xA5 || receive_header.data[1] != 0xA5)
	{
		//std:: cout<<receive_header.data[0]<<std::endl;
		//std:: cout<<receive_header.data[1]<<std::endl;

		ROS_ERROR("Received message header11 error!");
        	return false;
	}

	
	
    if (buf[2] != 0x03 || buf[3] != 0x08)
	{
		ROS_ERROR("Received message header error!");
        return false;
	}

	checkSum = Get_Sum(buf+2, 9);
    
    
    // 检查信息校验值
	if (checkSum != buf[11])
	{
		ROS_ERROR("Received data check sum error!");
        	return false;
	}

    // 读取速度值
    leftdataINT16.data[0]  = buf[5];
    leftdataINT16.data[1]  = buf[6];
    rightdataINT16.data[0] = buf[7];
    rightdataINT16.data[1] = buf[8];
	yawdataINT16.data[0] = buf[9];
    yawdataINT16.data[1] = buf[10];


	

    // 积分计算里程计信息
    //vx_  = (vel_right.odoemtry_float + vel_left.odoemtry_float) / 2 / 1000;
    //vth_ = (vel_right.odoemtry_float - vel_left.odoemtry_float) / ROBOT_LENGTH;
	vx_  = leftdataINT16.d/ 1000.0;
	vth_ = rightdataINT16.d / 1000.0;
    vy_  = RobotV_y;
    curr_time = ros::Time::now();

	double dt = (curr_time - last_time_).toSec();
	double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
	double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
	double delta_th = vth_ * dt;

	x_ += delta_x;
	y_ += delta_y;
	th_ += delta_th;
	last_time_ = curr_time;               

	return true;
}





void MRobot::writeSpeed(double RobotV_x, double RobotV_y,double YawRate)
{
	/*unsigned char buf[16] = {0};
	int i, length = 0;
	double r = RobotV / YawRate;


    // 设置消息头
	for(i = 0; i < 2; i++)
		buf[i] = header[i];
	
    // 设置消息类型
	send_command.d = GET_SPEED;
	for(i = 0; i < 2; i++)
		buf[i + 2] = send_command.data[i];
	
    // 设置机器人左右轮速度
	length = 8;
	buf[4] = length;

	leftdata.d = (int)(RobotV);
	rightdata.d = (int)(YawRate);
	for(i = 0; i < 4; i++)
	{
		buf[i + 5] = leftdata.data[i];
		buf[i + 9] = rightdata.data[i];
	}
    
    // 设置校验值、消息尾
	buf[5 + length] = getCrc8(buf, 5 + length);
	buf[6 + length] = ender[0];
	buf[6 + length + 1] = ender[1];*/



    unsigned char buf[16] = {0};
    unsigned char SUM = 0;

	buf[0] = 0x5A;
	buf[1] = 0x5A;
	buf[2] = 0x03;
	buf[3] = 0x08;
	buf[4] = 0x01;
	double temp=RobotV_x;
	double RobotV_y_t;
	RobotV_x = sqrt(RobotV_x*RobotV_x+RobotV_y*RobotV_y)*1000;
	if (RobotV_y>0)
	{RobotV_y_t = atan2(RobotV_y,temp)*57.29578;}
	else
	{RobotV_y_t = atan2(-RobotV_y,-temp)*57.29578+180;}

	leftdataINT16.d = (short)(RobotV_x);
	rightdataINT16.d = (short)(YawRate);
	yawdataINT16.d = (short)(RobotV_y_t);
	buf[5] = leftdataINT16.data[0];
	buf[6] = leftdataINT16.data[1];

	buf[7] = rightdataINT16.data[0];
	buf[8] = rightdataINT16.data[1];
	
	buf[9] = yawdataINT16.data[0];
	buf[10] = yawdataINT16.data[1];


	SUM = Get_Sum(buf+2, 9);
	buf[11] = SUM;

	//ROS_INFO("publishing");
   
    // 通过串口下发数据
	boost::asio::write(sp, boost::asio::buffer(buf));
}

bool MRobot::spinOnce(double RobotV_x,double RobotV_y, double YawRate)
{	
    
    // 下发机器人期望速度
    writeSpeed(RobotV_x, RobotV_y,YawRate);

    // 读取机器人实际速度
    readSpeed(RobotV_y);



    // 发布TF
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id  = "base_link";

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromYaw(th_);
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    odom_broadcaster_.sendTransform(odom_trans);

    // 发布里程计消息
    nav_msgs::Odometry msgl;
    msgl.header.stamp = ros::Time::now();
    msgl.header.frame_id = "odom";

    msgl.pose.pose.position.x = x_;
    msgl.pose.pose.position.y = x_;
    msgl.pose.pose.position.z = 0.0;
    msgl.pose.pose.orientation = odom_quat;
    msgl.pose.covariance = odom_pose_covariance;

    msgl.child_frame_id = "base_link";
    msgl.twist.twist.linear.x = vx_;
    msgl.twist.twist.linear.y = vy_;
    msgl.twist.twist.angular.z = vth_;
    msgl.twist.covariance = odom_twist_covariance;
  
    pub_.publish(msgl);
}

unsigned char MRobot::getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}

}
