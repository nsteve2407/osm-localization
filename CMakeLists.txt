cmake_minimum_required(VERSION 3.0.2)
project(osm-localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # geometry_msgs/Vector3
  # nav_msgs/Odometry
  roscpp
  rospy
  # sensor_msgs/PointCloud2
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pf_node
#  CATKIN_DEPENDS geometry_msgs/Vector3 nav_msgs/Odometry roscpp rospy sensor_msgs/PointCloud2
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


find_package(xtensor REQUIRED)

add_executable(pf_node src/pf_node.cpp)
target_link_libraries(pf_node ${catkin_LIBRARIES} -xc++ ${xtensor_LIBRARIES})
