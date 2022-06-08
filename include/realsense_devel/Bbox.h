/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: April 2022                        
 * File: Bbox.h                          */

#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include<iostream>
#include "ros/ros.h"
#include<realsense_devel/ClustersArray.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// Markers
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include<realsense_devel/BoundingBox3D.h>

class BoundingBox_moi {
    private:
    ros::Publisher bbox_pub;
    ros::Publisher bbox_markers_pub;
    ros::Subscriber clusters_sub;
    // reference frame
    std::string reference_frame;
public:
    void clusters_callback(const realsense_devel::ClustersArray::ConstPtr& clusters_msg);
    // function to find BBOX
    void getBBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, int j, visualization_msgs::Marker &marker, visualization_msgs::Marker &text_marker, realsense_devel::BoundingBox3D &bbox);
    //std:: tuple<visualization_msgs::Marker, visualization_msgs::Marker> getBBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, int j);
    // Constructor
    BoundingBox_moi(ros::NodeHandle *n);
    // Destructor
    ~BoundingBox_moi();
    };
    
    
class BoundingBox_pca {
private:
    ros::Publisher bbox_pub;
    ros::Publisher bbox_markers_pub;
    ros::Subscriber clusters_sub;
    // reference frame
    std::string reference_frame;
public:
    void clusters_callback(const realsense_devel::ClustersArray::ConstPtr& clusters_msg);
    // function to find BBOX
    void getBBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, int j, visualization_msgs::Marker &marker, visualization_msgs::Marker &text_marker, realsense_devel::BoundingBox3D &bbox);
    // Constructor
    BoundingBox_pca(ros::NodeHandle *n);
    // Destructor
    ~BoundingBox_pca();
};

#endif