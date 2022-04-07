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

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud){

  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // VoxelGrid
//  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
//  voxel_grid.setInputCloud (cloud);
//  voxel_grid.setLeafSize (0.03, 0.03, 0.03);
//  voxel_grid.filter (*cloud_filtered);
  
  *cloud_filtered = *cloud;
  
  // PassThrough
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloud_filtered);
//  pass.setFilterFieldName ("x");
//  pass.setFilterLimits (-1.0, 1.0);
//  pass.setFilterFieldName ("y");
//  pass.setFilterLimits (-1.0, 1.0);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  

  // Segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr indices (new pcl::PointIndices);
  // Create the segmentation object

  pcl::SACSegmentation<pcl::PointXYZ> seg;
/*  pcl::SACSegmentation<pcl::PCLPointCloud2> seg; // errors raise with <pcl::PCLPointCloud2> type
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  
  seg.setInputCloud(cloud_filtered);
  seg.segment(*indices, *coefficients);
  
  if (indices->indices.size() == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  } else {
      // Create the filtering object
        pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*cloud_filtered);
      }*/


    

  // Publish the data
  pub.publish (cloud_filtered);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}


