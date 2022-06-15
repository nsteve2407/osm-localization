cmake_minimum_required(VERSION 3.0.2)
project(osm_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # geometry_msgs/Vector3
  # nav_msgs/Odometry
  cv_bridge
  roscpp
  rospy
  pcl_ros
  message_filters
  message_generation
  geometry_msgs
  # sensor_msgs/PointCloud2
)
find_package(
  xtensor REQUIRED
  PCL REQUIRED
  Boost REQUIRED
  OpenCV REQUIRED
  
  )
add_service_files(  
  FILES
  GlobalSearch.srv
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)


find_package (Eigen3 3.3 REQUIRED NO_MODULE)
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp
                 sensor_msgs
                 
                 
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
  ${OpenCV_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})
add_library(osmpf
  /home/mkz/catkin_ws/src/osm-localization/src/osm_pf.cpp
  /home/mkz/catkin_ws/src/osm-localization/src/osmv2.cpp
)
# add_library(osmpf2 /home/mkz/catkin_ws/src/osm-localization/src/osmv2.cpp)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(pf_node src/pf_node.cpp)
target_link_libraries(pf_node osmpf ${catkin_LIBRARIES} -xc++ 
${xtensor_LIBRARIES} 
${PCL_LIBRARIES}
${Boost_LIBRARIES}
${Eigen3_LIBRARIES}
Eigen3::Eigen
${OpenCV_INCLUDE_DIRS}
${OpenCV_LIBS}
${OpenCV_LIBRARIES})

add_executable(pose_cov src/pose_cov.cpp)
target_link_libraries(pose_cov ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(img_to_pointcloud src/img_to_pointcloud.cpp)
target_link_libraries(img_to_pointcloud osmpf ${catkin_LIBRARIES} -xc++ 
${xtensor_LIBRARIES} 
${PCL_LIBRARIES}
${Boost_LIBRARIES}
${Eigen3_LIBRARIES}
Eigen3::Eigen
${OpenCV_INCLUDE_DIRS}
${OpenCV_LIBS}
${OpenCV_LIBRARIES})

add_executable(pf_v2_main src/v2/pf_v2_main.cpp)
target_link_libraries(pf_v2_main osmpf ${catkin_LIBRARIES} -xc++ 
${xtensor_LIBRARIES} 
${PCL_LIBRARIES}
${Boost_LIBRARIES}
${Eigen3_LIBRARIES}
Eigen3::Eigen
${OpenCV_INCLUDE_DIRS}
${OpenCV_LIBS}
${OpenCV_LIBRARIES})