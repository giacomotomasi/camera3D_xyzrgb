#include<iostream>

#include<ros/ros.h>
#include<realsense_devel/ClustersArray.h>
#include<sensor_msgs/PointCloud2.h>
// #include<realsense_devel/BoundingBox3D.h>  // I could not include it from vision_msgs package so I copied it in realsense_devel
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


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
            visualization_msgs::Marker marker, text_marker;
            tie(marker, text_marker) = getBBox(cloud, i);
            (*bbox_markers).markers.push_back(marker);
            (*bbox_markers).markers.push_back(text_marker);
            }
        bbox_pub.publish(bbox_markers);
    }


    // function to find BBOX
    std:: tuple<visualization_msgs::Marker, visualization_msgs::Marker> getBBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, int j){
        
        pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
        feature_extractor.setInputCloud(cluster);
        feature_extractor.compute();

        std::vector<float> moment_of_inertia;
        std::vector<float> eccentricity;
        pcl::PointXYZRGB min_point_OBB;
        pcl::PointXYZRGB max_point_OBB;
        pcl::PointXYZRGB position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;

        feature_extractor.getMomentOfInertia(moment_of_inertia);
        feature_extractor.getEccentricity(eccentricity);
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        feature_extractor.getEigenValues(major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter(mass_center);
        
        Eigen::Quaternionf quat(rotational_matrix_OBB);
            
        // create marker correspoinding to the bbox
        visualization_msgs::Marker marker;
        
        marker.header.frame_id = "camera_depth_optical_frame";
//        std::stringstream obs;
//        obs << "Obstacle " << j;
        marker.ns = "Obstacle";
        marker.header.stamp = ros::Time::now();
        marker.id = j;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0);
        marker.pose.position.x = position_OBB.x;
        marker.pose.position.y = position_OBB.y;
        marker.pose.position.z = position_OBB.z;
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker.pose.orientation.w = quat.w();
        marker.scale.x = max_point_OBB.x - min_point_OBB.x + 0.02;
        marker.scale.y = max_point_OBB.y - min_point_OBB.y + 0.02;
        marker.scale.z = max_point_OBB.z - min_point_OBB.z + 0.02;
        marker.color.a = 0.15;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0;
//        std::cout << "diff x " << j << " " << max_point_OBB.x - min_point_OBB.x << std::endl;
//        std::cout << "diff y " << j << " " << max_point_OBB.y - min_point_OBB.y << std::endl;
//        std::cout << "diff z " << j << " " << max_point_OBB.z - min_point_OBB.z << std::endl;
        
        // create TEXT marker
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "camera_depth_optical_frame";
        std::stringstream obs;
        obs << "Obstacle " << j;
        text_marker.text = obs.str();
        text_marker.ns = "Obstacle";
        text_marker.header.stamp = ros::Time::now();
        text_marker.id = j+100;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.lifetime = ros::Duration(0);
        text_marker.pose.position.x = position_OBB.x;
        text_marker.pose.position.y = position_OBB.y - ((max_point_OBB.x - min_point_OBB.x)/2 + 0.04);
        text_marker.pose.position.z = position_OBB.z;
        text_marker.pose.orientation.x = quat.x();
        text_marker.pose.orientation.y = quat.y();
        text_marker.pose.orientation.z = quat.z();
        text_marker.pose.orientation.w = quat.w();
        text_marker.scale.x = 0.04;
        text_marker.scale.y = 0.04;
        text_marker.scale.z = 0.04;
        text_marker.color.a = 1.0;
        text_marker.color.r = 1.0;
        text_marker.color.g = 0.0;
        text_marker.color.b = 0;
        
        return {marker, text_marker};
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
    
    
