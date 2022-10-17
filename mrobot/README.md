# mrobot ros package by ROSClub
## 环境
系统环境：<br>
ubuntu14.04 + ros-indigo-desktop<br>
ubuntu16.04 + ros-kinetic-desktop<br>
推荐组合：<br>
NVIDIA Jetson TK1 （ubuntu14.04 + ros-indigo-desktop）<br>
树莓派3 （ubuntu16.04 + ros-kinetic-desktop）<br>
## 工作空间
mkdir -p mrobot_ws/src<br>
cd ..<br>
catkin_init_workspace<br>
cd src<br>
git clone https://github.com/ROSClub/mrobot.git<br>
cd ~/mrobot_ws && catkin_make<br>
## STM32通信协议
⼀帧数据为： [消息头(2字节)] [命令(2字节)] [⻓度(1字节)] [数据(n字节， n=⻓度)] [校验(1字节)] [消息尾(2字节)]
消息头固定为[0x55 0xaa]，消息尾固定为[0x0d 0x0a]<br>
### 主控发送⽤命令参数
(1)0x5a 0x5a： 发送速度信息和电池信息<br>
(2)0x5a 0x55： 发送速度信息，电池信息和超声波信息<br>
(3)0x5a 0xaa： 发送速度信息，电池信息和六轴传感器信息<br>
(4)0x5a 0xa5： 发送速度信息，电池信息，超声波信息和六轴传感器信息<br>
(5)0xa5 0x5a： 发送速度信息<br>
(6)0xa5 0x55： 发送电池信息<br>
(7)0xa5 0xaa： 发送超声波信息<br>
(8)0xa5 0xa5： 发送六轴传感器信息<br>
### 主控接收⽤命令参数
(1)0x55 0xaa： 请求发送速度信息和电池信息<br>
(2)0x55 0x55： 请求发送速度信息，电池信息和超声波信息<br>
(3)0x55 0xa5： 请求发送速度信息，电池信息和六轴传感器信息<br>
(4)0x55 0x5a： 请求发送速度信息，电池信息，超声波信息和六轴传感器信息<br>
(5)0xaa 0xaa： 请求发送速度信息<br>
(6)0xaa 0x55： 请求发送电池信息<br>
(7)0xaa 0xa5： 请求发送超声波信息<br>
(8)0xaa 0x5a： 请求发送六轴传感器信息<br>
详细协议见使用手册通信协议章节，或者查看源代码<br>
## USB绑定
见使用手册3.4节，购买小车都会赠送详细的使用手册，及其大量资料。<br>
## 主从机配置
⼿动配置略麻烦，新⼿不友好。<br>
在远程主机使⽤我们提供的⼩⼯具⼀键配置即可<br>
cd ~/mrobot_ws/src/mrobot/startup/Bin<br>
./ToolBox<br>
## 串口工具
主要⽤来测试⼩⻋数据是否正常，以及⼤家更改源码后验证使⽤。<br>
cd ~/mrobot_ws/src/mrobot/startup/Bin<br>
./ToolBox<br>
## 基础操作
1、键盘控制实体机器人<br>
roslaunch mrobot_bringup mrobot_core.launch<br>
roslaunch mrobot_teleop mrobot_teleop_key.launch<br>
2、键盘控制实体机器人并查看<br>
roslaunch mrobot_bringup mrobot_robot.launch<br>
roslaunch mrobot_bringup mrobot_model.launch<br>
roslaunch mrobot_teleop mrobot_teleop_key.launch<br>
3、SLAM<br>
传感器为rplidar A2<br>
roslaunch mrobot_bringup mrobot_robot.launch<br>
roslaunch mrobot_slam mrobot_slam.launch<br>
rosrun rviz rviz -d `rospack find mrobot_slam`/rviz/mrobot_slam.rviz<br>
roslaunch mrobot_teleop mrobot_teleop_key.launch<br>
4、Navigation<br>
roslaunch mrobot_bringup mrobot_robot.launch<br>
roslaunch mrobot_navigation mrobot_navigation.launch map_file:=/home/m1/map/
20170826.yaml<br>
rosrun rviz rviz -d `rospack find mrobot_navigation`/rviz/mrobot_nav.rviz<br>
5、其他<br>
时间控制⼩⻋前进⼀⽶并返回<br>
rosrun mrobot_bringup timed_out_and_back.py<br>
时间控制⼩⻋前进⼀⽶并返回<br>
rosrun mrobot_bringup odom_out_and_back.py<br>
在地⾯⾏⾛⼀个边⻓为⼀⽶的正⽅格<br>
rosrun mrobot_bringup nav_square.py<br>
使⽤PS2遥控控制⼩⻋<br>
roslaunch mrobot_teleop mrobot_teleop_joystick.launch<br>

## 待续。。
