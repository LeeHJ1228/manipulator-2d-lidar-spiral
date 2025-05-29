cmake_minimum_required(VERSION 3.0.2)
project(spiral)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  message_filters
  std_msgs
)
find_package(PCL REQUIRED COMPONENTS common io)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}  # PCL 경로 추가
)
add_executable(spiral_node src/spiral_node.cpp)
target_link_libraries(spiral_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}  # PCL 라이브러리 추가
)
