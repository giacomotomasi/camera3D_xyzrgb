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
    ros::init(argc, argv, "test_bbox_node");
    
    ros::NodeHandle n;

    // cloud_cluster_0 --> bottle | cloud_cluster_1 --> box
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/giacomo/Desktop/bagfiles/new_cloud_cluster_0.pcd", *cloud0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/giacomo/Desktop/bagfiles/new_cloud_cluster_1.pcd", *cloud1);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds {cloud0, cloud1};
// ======================================================================================

pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    for (int i {0}; i<clouds.size(); i++){
        pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
        feature_extractor.setInputCloud(clouds.at(i));
        feature_extractor.compute();

        std::vector <float> moment_of_inertia;
        std::vector <float> eccentricity;
        // pcl::PointXYZRGB min_point_AABB;
        // pcl::PointXYZRGB max_point_AABB;
        pcl::PointXYZRGB min_point_OBB;
        pcl::PointXYZRGB max_point_OBB;
        pcl::PointXYZRGB position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;

        feature_extractor.getMomentOfInertia(moment_of_inertia);
        feature_extractor.getEccentricity(eccentricity);
        // feature_extractor.getAABB(min_point_AABB, max_point_AABB);
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        feature_extractor.getEigenValues(major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter(mass_center);

        viewer->setBackgroundColor(0,0,0); // (1,1,1) white background
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        std::stringstream cloud_id;
        cloud_id << "cloud_" << i;
        viewer->addPointCloud<pcl::PointXYZRGB>((clouds.at(i)), cloud_id.str());
//        viewer->addPointCloud<pcl::PointXYZRGB>(cloud1, "sample cloud 1");
        // viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
        // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

        Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Quaternionf quat(rotational_matrix_OBB);
        viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, cloud_id.str());
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cloud_id.str());

        pcl::PointXYZRGB center(mass_center(0), mass_center(1), mass_center(2));
        pcl::PointXYZRGB x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
        pcl::PointXYZRGB y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
        pcl::PointXYZRGB z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
        std::stringstream major_eigen_vector;
        major_eigen_vector << "major eigen vector " << i;
        std::stringstream middle_eigen_vector;
        middle_eigen_vector << "middle eigen vector " << i;
        std::stringstream minor_eigen_vector;
        minor_eigen_vector << "minor eigen vector " << i;
        viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, major_eigen_vector.str());
        viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, middle_eigen_vector.str());
        viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, minor_eigen_vector.str());
    }
    while(!viewer->wasStopped()){
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
    
    ros::spin();
}