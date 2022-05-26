#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/transforms.h>

ros::Publisher pub;
std::string target_link {"base_link"};

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    
    tf::StampedTransform transform;
    tf::TransformListener listener;
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/base_link", "/camera_link", now, ros::Duration(3.0));
    listener.lookupTransform("/base_link", "/camera_link", ros::Time(0), transform);
    
    sensor_msgs::PointCloud2 cloud_transformed;
    
    pcl_ros::transformPointCloud(target_link, *cloud_msg, cloud_transformed, listener);
    cloud_transformed.header.stamp = ros::Time::now();
    cloud_transformed.header.frame_id = "base_link";
    pub.publish(cloud_transformed);
    }

int main(int argc, char** argv){
  ros::init(argc, argv, "camera_tf_listener");

  ros::NodeHandle node;
  std::cout << "Camera TF broadcaster created!" << std::endl;

  pub = node.advertise<sensor_msgs::PointCloud2>("pcl_transformed", 1);
  ros::Subscriber sub = node.subscribe("/camera/depth/color/points", 1, cloud_callback);

  ros::spin();
  std::cout << "Camera TF broadcaster closed!" << std::endl;
  
  return 0;
};