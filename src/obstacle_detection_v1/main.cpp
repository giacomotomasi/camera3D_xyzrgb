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

class Detector {
private:
    ros::Publisher cloud_pub;
    ros::Subscriber cloud_sub;
    // pointer to pcl::PointXYZRGB 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
public:
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
        // convert cloud to pcl::PointXYZRGB
        pcl::fromROSMsg (*cloud_msg, *cloud);
        // std::cout << "cloud converted" << std::endl;
        
        //  NOT THE BEST WAY (IN MY OPINION) --- FIND A BETTER (CLEAN) WAY
        // Detector::voxel_grid();
        Detector::pass_through();
        Detector::segmentation();
        Detector::outlier_removal();
        
        Detector::publish();
        }
        
    void voxel_grid(){
        // VoxelGrid
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
        voxel_grid.setInputCloud (cloud);
        voxel_grid.setLeafSize (0.05, 0.05, 0.05);
        voxel_grid.filter (*cloud);
        };
    
    void pass_through(){
        // PassThrough
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 1.5);
        pass.filter (*cloud);
        // std::cout << "Point Cloud width: %d " << cloud->width << std::endl;
        };
    
    void segmentation(){
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
            Detector::extract_indices(indices);
            }
        };
      
    void extract_indices(pcl::PointIndices::Ptr indices){
        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(indices);
        extract.setNegative(true);
        extract.filter(*cloud);
        };
        
    void outlier_removal(){
        // Outlier removal
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloud);
        };
        
    void publish(){
        // reconvert to PointCloud2 to be ROS compatible
        pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());
        pcl::toPCLPointCloud2(*cloud, *cloud_filtered);
        cloud_pub.publish(cloud_filtered);
        //std::cout << "Point Cloud width: %d " << cloud->width << std::endl;
        };
    
    // Constructor
    Detector(ros::NodeHandle *n1){
        std::cout << "Detector constructor called" << std::endl;
        // Create pointer in the heap
        cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        // Create a ROS subscriber for the input point cloud
        cloud_sub = n1->subscribe ("/camera/depth/color/points", 1, &Detector::cloud_callback, this);
        // Createa ROS publisher for the filtered point cloud
        cloud_pub = n1->advertise<pcl::PCLPointCloud2>("pcl_filtered", 1);
        };
    // Deconstructor
    ~Detector(){
        std::cout << "Detector deconstructor called" << std::endl;
        };
    };


int main(int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "cloud_filter_node");
    
    ros::NodeHandle n1;
    
    Detector d(&n1);
    
//    while (ros::ok()){
//        // std::cout << "I am inside the while loop" << std::endl;
//        d.pass_through();
//        d.publish();
//        }
    
    // Spin
    ros::spin ();
    
    return 0;
    }