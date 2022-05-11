#include <iostream>
#include "ros/ros.h"
#include "realsense_devel/Detector.h"

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "cloud_filter_node");
    
    ros::NodeHandle n1;
    Detector d(&n1);
    
    // Spin
    ros::spin ();
    
    return 0;
    }