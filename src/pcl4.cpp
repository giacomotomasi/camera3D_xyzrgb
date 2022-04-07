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
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    
    // convert to standard PCL type (some methods were not compatible with PointCloud2)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*cloud_msg, *cloud);
  
    // PassThrough
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.5);
    pass.filter (*cloud);
    
    // Segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // pcl::SACSegmentation<pcl::PCLPointCloud2> seg; // errors raise with <pcl::PCLPointCloud2> type
    // Optional
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
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(indices);
        extract.setNegative(true);
        extract.filter(*cloud);
      }
      
    // Outlier removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud);
    
    // reconvert to PointCloud2 to be ROS compatible
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud, *cloud_filtered);
    
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
  pub = nh.advertise<pcl::PCLPointCloud2> ("pcl_filtered", 1);

  // Spin
  ros::spin ();
}


