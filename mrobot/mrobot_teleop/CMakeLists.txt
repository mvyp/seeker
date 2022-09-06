cmake_minimum_required(VERSION 2.8.3)
project(mrobot_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mrobot_teleop_joystick
  src/mrobot_teleop_joystick.cpp
)

target_link_libraries(mrobot_teleop_joystick
  ${catkin_LIBRARIES}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   scripts/mrobot_teleop_key.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
