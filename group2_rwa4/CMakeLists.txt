cmake_minimum_required(VERSION 2.8.3)
project(group2_rwa4)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  osrf_gear
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  trajectory_msgs
  geometry_msgs
  moveit_ros_planning_interface
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS osrf_gear moveit_ros_planning_interface geometry_msgs std_msgs roscpp sensor_msgs std_srvs tf2_geometry_msgs trajectory_msgs
)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(kit_rwa4 src/main.cpp src/Competition.cpp src/Orders.cpp src/RobotController.cpp src/Sensors.cpp)

add_dependencies(kit_rwa4  ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(kit_rwa4 ${catkin_LIBRARIES})

install(FILES
         config/rwa4_sensors.yaml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
 )
