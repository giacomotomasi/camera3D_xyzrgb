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

class Detector {
private:
    ros::Publisher cloud_pub;
    ros::Publisher clusters_pub;
    ros::Subscriber cloud_sub;
    // pointer to pcl::PointXYZRGB 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster;
public:
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
        // convert cloud to pcl::PointXYZRGB
        pcl::fromROSMsg (*cloud_msg, *cloud);
        // std::cout << "cloud converted" << std::endl;
        
        //  NOT THE BEST WAY (IN MY OPINION) --- FIND A BETTER (CLEANER) WAY
        // Detector::voxel_grid();
        Detector::pass_through();
        Detector::segmentation();
        Detector::outlier_removal();
        Detector::cluster_extraction(); // includes publish method
        
        Detector::outlier_removal();
        
        Detector::publish();
        };
        
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
        pass.setFilterLimits (0.0, 1.0);
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
            Detector::extract_indices(indices, true);
            }
        };
      
    void extract_indices(pcl::PointIndices::Ptr indices, bool mode){
        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(indices);
        extract.setNegative(mode); // setNegative(true) extract the indices from cloud. setNegative(false) leaves only indices
        extract.filter(*cloud);
        };
        
    void outlier_removal(){
        // Outlier removal
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh (1.0);
        sor.filter(*cloud);
        };
        
    void cluster_extraction(){
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
        
//        int j = 0;
//        pcl::PCDWriter writer;
        pcl::PointIndices::Ptr indices (new pcl::PointIndices);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
              cloud_cluster->push_back((*cloud)[*pit]);
              // Merge indices form clusters
              indices->indices.push_back(*pit);
              }
              
//            std::stringstream ss;
//            ss << "/home/giacomo/Desktop/bagfiles/new_cloud_cluster_" << j << ".pcd";
//            writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);
//            j++;
            
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
            
//            pcl::PCLPointCloud2::Ptr cloud_ros (new pcl::PCLPointCloud2());
//            pcl::toPCLPointCloud2(*cloud_cluster, *cloud_ros);
//            (*cloud_ros).header.frame_id = "camera_depth_optical_frame";
//            cloud_pub.publish(cloud_ros);
            
            clusters_pub.publish(clusters_array);
        };

        
    void publish(){
        // reconvert to PointCloud2 to be ROS compatible
        pcl::PCLPointCloud2::Ptr cloud_ros (new pcl::PCLPointCloud2());
        pcl::toPCLPointCloud2(*cloud, *cloud_ros);
        cloud_pub.publish(cloud_ros);
        //std::cout << "Point Cloud width: %d " << cloud->width << std::endl;
        };

    
    // Constructor
    Detector(ros::NodeHandle *n1){
        std::cout << "\033[1;32m Detector constructor called.\033[0m" << std::endl; // print in green color
        // ROS_INFO("\033[1;32m Detector constructor called.\033[0m");
        // Create pointer in the heap
        cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        // cloud_cluster = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        // Create a ROS subscriber for the input point cloud
        cloud_sub = n1->subscribe ("/camera/depth/color/points", 1, &Detector::cloud_callback, this);
        // Create a ROS publisher for the filtered point cloud and for the clusters
        cloud_pub = n1->advertise<pcl::PCLPointCloud2>("pcl_filtered", 1);
        clusters_pub = n1->advertise<realsense_devel::ClustersArray>("pcl_clusters", 1);
        };
    // Destructor
    ~Detector(){
        std::cout << "\033[1;32m Detector deconstructor called.\033[0m" << std::endl; 
        // ROS_INFO("\033[1;32m Detector deconstructor called.\033[0m");
        };
    };


int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "cloud_filter_node");
    
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
    
    
    
    
    
