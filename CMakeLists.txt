
cmake_minimum_required(VERSION 2.8.3)
project(object_detection)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  pcl_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
  tf_conversions
)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ObjDet src/ObjDet.cpp)

target_link_libraries(ObjDet ${catkin_LIBRARIES} ${PCL_LIBRARIES})

