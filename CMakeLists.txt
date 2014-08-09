cmake_minimum_required(VERSION 2.8.3)
project(pcl_sandbox)

find_package(catkin REQUIRED roscpp  pcl_conversions pcl_ros sensor_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_offset
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(pcl_sandbox src/pcl_sandbox.cpp)

target_link_libraries(pcl_sandbox
   ${catkin_LIBRARIES}
)

