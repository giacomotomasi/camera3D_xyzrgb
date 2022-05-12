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
    if (voxel_grid_enabled)
        Detector::voxel_grid();
    if (pass_through_enabled)
        Detector::pass_through();
    if (segmentation_enabled)
        Detector::segmentation();
    if (outlier_removal_enabled)
        Detector::outlier_removal();
    if (cluster_extraction_enabled)
        Detector::cluster_extraction(); // includes publish method
    if (outlier_removal_enabled)
        Detector::outlier_removal();
    
    Detector::publish();
    }
    
void Detector::voxel_grid(){
    // VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud (cloud);
    voxel_grid.setLeafSize (size_x,size_y,size_z);
    voxel_grid.filter (*cloud);
    }

void Detector::pass_through(){
    // PassThrough
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min,z_max);
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
    seg.setDistanceThreshold (distance_threshold);
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
    sor.setMeanK(meanK);
    sor.setStddevMulThresh (standard_dev_mult);
    sor.filter(*cloud);
    }
    
void Detector::cluster_extraction(){
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(cluster_tolerance); // 2cm
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
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
    // get ros parameters
    n1->param<std::string>("/pointcloud_topic/camera_topic",pointcloud_topic,"/camera/depth/color/points");
    std::cout << "topic name " << pointcloud_topic << std::endl;
    n1->param("/voxel_grid/x",size_x,0.05);
    n1->param("/voxel_grid/y",size_y,0.05);
    n1->param("/voxel_grid/z",size_z,0.05);
    n1->param("/voxel_grid/enable",voxel_grid_enabled,false);
    n1->param("/pass_through/z_min",z_min,0.0);
    n1->param("/pass_through/z_max",z_max,1.0);
    n1->param("/pass_through/enable",pass_through_enabled,true);
    n1->param("/segmentation/distance_threshold",distance_threshold,0.01);
    n1->param("/segmentation/enable",segmentation_enabled,true);
    n1->param("/outlier_removal/meanK",meanK,50);
    n1->param("/outlier_removal/standard_dev_mult",standard_dev_mult,1.0);
    n1->param("/outlier_removal/enable",outlier_removal_enabled,true);
    n1->param("/cluster_extraction/cluster_tolerance",cluster_tolerance,0.02);
    n1->param("/cluster_extraction/min_cluster_size",min_cluster_size,1000);
    n1->param("/cluster_extraction/max_cluster_size",max_cluster_size,25000);
    n1->param("/cluster_extraction/enable",cluster_extraction_enabled,true);
    // Create pointer in the heap
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Create a ROS subscriber for the input point cloud
    cloud_sub = n1->subscribe (pointcloud_topic, 1, &Detector::cloud_callback, this);
    // Create a ROS publisher for the filtered point cloud and for the clusters
    cloud_pub = n1->advertise<pcl::PCLPointCloud2>("pcl_filtered", 1);
    clusters_pub = n1->advertise<realsense_devel::ClustersArray>("pcl_clusters", 1);
    }
// Destructor
Detector::~Detector(){
    std::cout << "\033[1;32m Detector deconstructor called.\033[0m" << std::endl;
    }