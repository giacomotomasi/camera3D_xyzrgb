#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/transforms.h>

ros::Publisher pub;
tf::StampedTransform transform;
tf::TransformListener listener;
std::string target_link {"base_link"};

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    
    listener.lookupTransform("/base_link", "/camera_link", ros::Time(0), transform);
    
    sensor_msgs::PointCloud2 cloud_transformed;
    
    pcl_ros::transformPointCloud(target_link, *cloud_msg, cloud_transformed, listener);
    pub.publish(cloud_transformed);
    }

int main(int argc, char** argv){
  ros::init(argc, argv, "camera_tf_listener");

  ros::NodeHandle node;

  pub = node.advertise<sensor_msgs::PointCloud2>("pcl_transformed", 1);
  ros::Subscriber sub = node.subscribe("/camera/depth/color/points", 1, cloud_callback);

  ros::Rate rate(10.0);
  
  return 0;
};