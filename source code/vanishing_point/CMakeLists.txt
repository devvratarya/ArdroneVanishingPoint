cmake_minimum_required(VERSION 2.8.3)
project(vanishing_point)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  tum_ardrone
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ardrone_autonomy tum_ardrone roscpp std_msgs geometry_msgs image_transport
)

include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
include_directories(include)
# Declare a cpp executable
add_executable(simplecanny_node src/simplecanny.cpp src/main_simplecanny.cpp)
#add_executable(simplecanny_node src/simplecanny.cpp)
add_executable(densityvp_node src/densityVP.cpp)
#add_executable(vanishingpt_det_node src/vanishingpoint_det.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(simplecanny_node ${OpenCV_LIBRARIES} 
   ${catkin_LIBRARIES}
 )

target_link_libraries(densityvp_node ${OpenCV_LIBRARIES} 
   ${catkin_LIBRARIES}
 )

 #target_link_libraries(vanishingpt_det_node ${OpenCV_LIBRARIES} 
 #  ${catkin_LIBRARIES}
 #)


