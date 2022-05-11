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
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(detector_node src/detector_node.cpp src/Detector.cpp)
target_link_libraries(detector_node ${catkin_LIBRARIES})

add_executable(bbox_node src/bbox_node.cpp src/Bbox.cpp)
target_link_libraries(bbox_node ${catkin_LIBRARIES})

#add_executable(bbox_pca src/bbox_pca.cpp)
#target_link_libraries(bbox_pca ${catkin_LIBRARIES})

#add_executable(bbox_moi src/bbox_moi.cpp)
#target_link_libraries(bbox_moi ${catkin_LIBRARIES})