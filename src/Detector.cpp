/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: April 2021                        
 * File: Detector.cpp                          */
 
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// PCL Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <realsense_devel/ClustersArray.h>
#include "realsense_devel/Detector.h"

void Detector::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    // convert cloud to pcl::PointXYZRGB
    pcl::fromROSMsg (*cloud_msg, *cloud);
    // Detector::voxel_grid();
    Detector::pass_through();
    Detector::segmentation();
    Detector::outlier_removal();
    Detector::cluster_extraction(); // includes publish method
    Detector::outlier_removal();
    Detector::publish();
    }
    
void Detector::voxel_grid(){
    // VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud (cloud);
    voxel_grid.setLeafSize (0.05, 0.05, 0.05);
    voxel_grid.filter (*cloud);
    }

void Detector::pass_through(){
    // PassThrough
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*cloud);
    }

void Detector::segmentation(){
    // Segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud(cloud);
    seg.segment(*indices, *coefficients);
    if (indices->indices.size() == 0)
    {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    } else {
        // Create the filtering object
        Detector::extract_indices(indices, true);
        }
    }
  
void Detector::extract_indices(pcl::PointIndices::Ptr indices, bool mode){
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(mode); // setNegative(true) extract the indices from cloud. setNegative(false) leaves only indices
    extract.filter(*cloud);
    }
    
void Detector::outlier_removal(){
    // Outlier removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh (1.0);
    sor.filter(*cloud);
    }
    
void Detector::cluster_extraction(){
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(1000);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    realsense_devel::ClustersArray clusters_array;
    
    // Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. 
    // To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, 
    // create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
          cloud_cluster->push_back((*cloud)[*pit]);
          // Merge indices form clusters
          indices->indices.push_back(*pit);
          }
        
        // Publish each cluster as a pointcloud2 message in a PoinCloud2 Array
        pcl::PCLPointCloud2::Ptr cloud_ros (new pcl::PCLPointCloud2());
        // convert to pcl::PCLPointCloud2
        pcl::toPCLPointCloud2(*cloud_cluster, *cloud_ros);
        (*cloud_ros).header.frame_id = "camera_depth_optical_frame";
        ros::Time time_st = ros::Time::now ();
        (*cloud_ros).header.stamp = time_st.toNSec()/1e3;
        sensor_msgs::PointCloud2 output;
        // Convert to ROS data type
        pcl_conversions::fromPCL(*cloud_ros, output);
        clusters_array.clusters.push_back(output);
        }
        
        Detector::extract_indices(indices, false);
        clusters_pub.publish(clusters_array);
    }
    
void Detector::publish(){
    // reconvert to PointCloud2 to be ROS compatible
    pcl::PCLPointCloud2::Ptr cloud_ros (new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud, *cloud_ros);
    cloud_pub.publish(cloud_ros);
    //std::cout << "Point Cloud width: %d " << cloud->width << std::endl;
    }

// Constructor
Detector::Detector(ros::NodeHandle *n1){
    std::cout << "\033[1;32m Detector constructor called.\033[0m" << std::endl; // print in green color
    // Create pointer in the heap
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Create a ROS subscriber for the input point cloud
    cloud_sub = n1->subscribe ("/camera/depth/color/points", 1, &Detector::cloud_callback, this);
    // Create a ROS publisher for the filtered point cloud and for the clusters
    cloud_pub = n1->advertise<pcl::PCLPointCloud2>("pcl_filtered", 1);
    clusters_pub = n1->advertise<realsense_devel::ClustersArray>("pcl_clusters", 1);
    }
// Destructor
Detector::~Detector(){
    std::cout << "\033[1;32m Detector deconstructor called.\033[0m" << std::endl;
    }