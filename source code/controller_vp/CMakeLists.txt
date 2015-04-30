cmake_minimum_required(VERSION 2.8.3)
project(controller_vp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()


install(PROGRAMS nodes/ArdroneFollower.py
		nodes/ArdroneController.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

