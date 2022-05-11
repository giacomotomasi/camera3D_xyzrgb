#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include<iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class Detector {
private:
    ros::Publisher cloud_pub;
    ros::Publisher clusters_pub;
    ros::Subscriber cloud_sub;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
public:
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void voxel_grid();
    void pass_through();
    void segmentation();
    void extract_indices(pcl::PointIndices::Ptr indices, bool mode);
    void outlier_removal();
    void cluster_extraction();
    void publish();
    // Constructor
    Detector(ros::NodeHandle *n1);
    // Destructor
    ~Detector();
    };

#endif