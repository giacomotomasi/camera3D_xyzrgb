cmake_minimum_required(VERSION 3.0.2)
project(realsense_devel)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(points_subscriber src/points_subscriber.cpp)
target_link_libraries(points_subscriber ${catkin_LIBRARIES})

add_executable(pcl1 src/pcl1.cpp)
target_link_libraries(pcl1 ${catkin_LIBRARIES})

add_executable(pcl2 src/pcl2.cpp)
target_link_libraries(pcl2 ${catkin_LIBRARIES})

add_executable(pcl3 src/pcl3.cpp)
target_link_libraries(pcl3 ${catkin_LIBRARIES})

add_executable(pcl4 src/pcl4.cpp)
target_link_libraries(pcl4 ${catkin_LIBRARIES})




