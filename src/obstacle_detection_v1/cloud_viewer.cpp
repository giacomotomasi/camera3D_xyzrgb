#include <iostream>
#include <vector>
#include <thread>

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/moment_of_inertia_estimation.h>

using namespace std::chrono_literals;

int main(int argc, char** argv){
    
    // ROS initialization
    ros::init(argc, argv, "cloud_viewer_node");
    
    ros::NodeHandle n;

    // cloud_cluster_0 --> bottle | cloud_cluster_1 --> box
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/giacomo/Desktop/bagfiles/Captured_Frame0.pcd", *cloud);
// ======================================================================================

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
        
    while(!viewer->wasStopped()){
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
    
    ros::spin();
}