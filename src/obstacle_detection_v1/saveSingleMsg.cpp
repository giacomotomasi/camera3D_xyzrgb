#include <iostream>
#include <ros/ros.h>
#include <realsense_devel/ClustersArray.h>
#include <rosbag/bag.h>


int n {0};

void clusters_callback(const realsense_devel::ClustersArray::ConstPtr& clusters_msg){
    ROS_INFO("I have received a ClustersArray mesage");
    n++;
    if (n == 2){
        ROS_INFO("Saving ClustersArray message to .bag file..");
        rosbag::Bag bag;
        bag.open("/home/giacomo/Desktop/bagfiles/clustersArrayMsg.bag", rosbag::bagmode::Write);
        bag.write("pcl_clusters", ros::Time::now(), clusters_msg);
        bag.close();
        ROS_INFO("Message saved!");
        ROS_WARN("Shutting downd saveSingleMsg node!");
        ros::shutdown();
        }
    }

int main(int argc, char** argv){
    ros::init(argc, argv, "saveSingleMsg_node");
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("/pcl_clusters",1, clusters_callback);
    
    ros::spin();
    
    }