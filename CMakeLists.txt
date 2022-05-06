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

add_executable(points_subscriber src/test/points_subscriber.cpp)
target_link_libraries(points_subscriber ${catkin_LIBRARIES})

add_executable(pcl1 src/test/pcl1.cpp)
target_link_libraries(pcl1 ${catkin_LIBRARIES})

add_executable(pcl2 src/test/pcl2.cpp)
target_link_libraries(pcl2 ${catkin_LIBRARIES})

add_executable(pcl3 src/test/pcl3.cpp)
target_link_libraries(pcl3 ${catkin_LIBRARIES})

add_executable(pcl4 src/test/pcl4.cpp)
target_link_libraries(pcl4 ${catkin_LIBRARIES})

add_executable(clustering1 src/obstacle_detection_v1/clustering1.cpp)
target_link_libraries(clustering1 ${catkin_LIBRARIES})

add_executable(bbox_backup src/obstacle_detection_v1/bbox_backup.cpp)
target_link_libraries(bbox_backup ${catkin_LIBRARIES})

add_executable(saveSingleMsg src/obstacle_detection_v1/saveSingleMsg.cpp)
target_link_libraries(saveSingleMsg ${catkin_LIBRARIES})

add_executable(test_bbox src/obstacle_detection_v1/test_bbox.cpp)
target_link_libraries(test_bbox ${catkin_LIBRARIES} pcl_visualization)

add_executable(cloud_viewer src/obstacle_detection_v1/cloud_viewer.cpp)
target_link_libraries(cloud_viewer ${catkin_LIBRARIES} pcl_visualization)




