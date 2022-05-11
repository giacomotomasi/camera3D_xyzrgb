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
  message_generation
  visualization_msgs
  tf
  tf_conversions
)

add_message_files(
  FILES
  ClustersArray.msg
  BoundingBox3D.msg
  
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(clustering1 src/obstacle_detection_v1/clustering1.cpp)
target_link_libraries(clustering1 ${catkin_LIBRARIES})

add_executable(bbox_pca src/obstacle_detection_v1/bbox_pca.cpp)
target_link_libraries(bbox_pca ${catkin_LIBRARIES})

add_executable(bbox_moi src/obstacle_detection_v1/bbox_moi.cpp)
target_link_libraries(bbox_moi ${catkin_LIBRARIES})






