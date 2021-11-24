cmake_minimum_required(VERSION 3.0.2)
project(osm-localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # geometry_msgs/Vector3
  # nav_msgs/Odometry
  roscpp
  rospy
  pcl_ros
  message_filters
  # sensor_msgs/PointCloud2
)
find_package(
  xtensor REQUIRED
  PCL REQUIRED
  Boost REQUIRED
  Eigen3 REQUIRED)

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
include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)
add_library(osmpf /home/mkz/catkin_ws/src/osm-localization/src/osm_pf.cpp)

add_executable(pf_node src/pf_node.cpp)
target_link_libraries(pf_node osmpf ${catkin_LIBRARIES} -xc++ 
${xtensor_LIBRARIES} 
${PCL_LIBRARIES}
${Boost_LIBRARIES}
${Eigen3_LIBRARIES})
