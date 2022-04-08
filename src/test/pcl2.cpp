#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud; // typedef in C/C++ is a keyword used to assign alternative names to the existing datatypes

void points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*msg, *cloud);
    // after this step, cloud points to a pcl pointCloud object (?)
    std::cout << "aaa" << std::endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pcl2");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/camera/depth/color/points", 1, points_callback);

  ros::spin();

  return 0;
}


