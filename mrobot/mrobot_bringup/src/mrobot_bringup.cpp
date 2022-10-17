#include "mrobot.h"
#include<cmath>
double RobotV_x = 0;
double RobotV_y = 0;
double YawRate_ = 0;

// 速度控制消息的回调函数
void cmdCallback(const geometry_msgs::Twist& msg)
{
	RobotV_x=msg.linear.x;
	RobotV_y =msg.linear.y;
	YawRate_ = msg.angular.z * 1000;

}
    
int main(int argc, char** argv)
{
    //初始化ROS节点
	ros::init(argc, argv, "mrobot_bringup");									
	ros::NodeHandle nh;
    
    //初始化MRobot
	mrobot::MRobot robot;
	if(!robot.init())
		ROS_ERROR("MRobot initialized failed.");
	ROS_INFO("MRobot initialized successful.");
    
	ros::Subscriber sub = nh.subscribe("cmd_vel", 50, &cmdCallback);

    //循环运行
	ros::Rate loop_rate(50);
	while (ros::ok()) 
	{
		ros::spinOnce();
        
        // 机器人控制
        robot.spinOnce(RobotV_x, RobotV_y, YawRate_);
        
		loop_rate.sleep();
	}

	return 0;
}


