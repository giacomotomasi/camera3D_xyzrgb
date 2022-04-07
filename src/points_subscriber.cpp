#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO("pointcloud height %d", msg->height);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "points_subscriber");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/camera/depth/color/points", 1, points_callback);

  ros::spin();

  return 0;
}
