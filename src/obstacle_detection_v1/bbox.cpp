#include<iostream>

#include<ros/ros.h>
#include<realsense_devel/ClustersArray.h>
// #include<sensor_msgs/PointCloud2.h>
// #include<realsense_devel/BoundingBox3D.h>  // I could not include it from vision_msgs package so I copied it in realsense_devel
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
// Markers
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// tf
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"



class BoundingBox{
private:
    ros::Publisher bbox_pub;
    ros::Subscriber clusters_sub;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
public:
    void clusters_callback(const realsense_devel::ClustersArray::ConstPtr& clusters_msg){
        visualization_msgs::MarkerArray::Ptr bbox_markers (new visualization_msgs::MarkerArray);
        //std::cout << (*clusters_msg).clusters.size() << std::endl;
        for (int i {0};i<(*clusters_msg).clusters.size();i++){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            // convert cloud to pcl::PointXYZRGB
            pcl::fromROSMsg((*clusters_msg).clusters.at(i), *cloud);
            (*bbox_markers).markers.push_back(getBBox(cloud, i));
            }
        bbox_pub.publish(bbox_markers);
    }


    // function to find BBOX
    visualization_msgs::Marker getBBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, int j){
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PCA<pcl::PointXYZRGB> pca;
        pca.setInputCloud(cluster);
        pca.project(*cluster, *projected_cloud);
        Eigen::Matrix3f eigen_vector_pca = pca.getEigenVectors();
        Eigen::Vector3f eigen_values = pca.getEigenValues();
        Eigen::Matrix3d eigen_vector_pca_double = eigen_vector_pca.cast<double>();
        pcl::PointXYZRGB min_point, max_point;
        pcl::getMinMax3D(*projected_cloud, min_point, max_point);
        Eigen::Vector4f cluster_centroid;
        pcl::compute3DCentroid(*cluster, cluster_centroid);
        tf::Quaternion quat;
        tf::Matrix3x3 tf_rotation;
        tf::matrixEigenToTF(eigen_vector_pca_double, tf_rotation);
        
        tf_rotation.getRotation(quat);
//        double roll, pitch, yaw;
//        tf_rotation.getRPY(roll, pitch, yaw);
            
        // create marker correspoinding to the bbox
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.header.frame_id = "camera_depth_optical_frame";
        std::stringstream obs;
        obs << "Obstacle " << j;
        marker.text = obs.str();
        marker.header.stamp = ros::Time::now();
        marker.id = j;

        marker.lifetime = ros::Duration(0);
        marker.pose.position.x = cluster_centroid(0);
        marker.pose.position.y = cluster_centroid(1);
        marker.pose.position.z = cluster_centroid(2);
        marker.pose.orientation.x = quat.getAxis().getX();
        marker.pose.orientation.y = quat.getAxis().getY();
        marker.pose.orientation.z = quat.getAxis().getZ();
        marker.pose.orientation.w = quat.getW();
        marker.scale.x = max_point.x - min_point.x;
        marker.scale.y = max_point.y - min_point.y;
        marker.scale.z = max_point.z - min_point.z;
        marker.color.a = 0.15;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0;
        
        return marker;
        }
    
    // Constructor
    BoundingBox(ros::NodeHandle *n){
        std::cout << "\033[1;32m BoundingBox constructor called.\033[0m" << std::endl;
        bbox_pub = n->advertise<visualization_msgs::MarkerArray>("bbox_marker", 1);
        clusters_sub = n->subscribe("pcl_clusters", 1, &BoundingBox::clusters_callback, this);
        }
    // Destructor
    ~BoundingBox(){
        std::cout << "\033[1;32m BoundingBox destructor called.\033[0m" << std::endl; 
        };
};

int main(int argc, char** argv){
    // ROS initialization
    ros::init(argc, argv, "bbox_node");
    
    ros::NodeHandle n;
//    
    BoundingBox b(&n);
    
    ros::spin();
    return 0;
    }
    
    
