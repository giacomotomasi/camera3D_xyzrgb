/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: April 2021                        
 * File: bbox_node.cpp                          */
 
#include <iostream>
#include "ros/ros.h"
#include "realsense_devel/Bbox.h"

int main(int argc, char** argv){
    // ROS initialization
    ros::init(argc, argv, "bbox_node");
    
    ros::NodeHandle n;
    // Momento of inertia method
    BoundingBox_moi b(&n);
    // Principal Component Analisys method
    // BoundingBox_pca b(&n);
    
    ros::spin();
    return 0;
    }