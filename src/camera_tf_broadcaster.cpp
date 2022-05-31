#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "camera_tf_broadcaster");

    ros::NodeHandle node;
    ros::Rate rate(150);
    double x {}, y {}, z {}, roll {}, pitch {}, yaw {};
    
    node.param("/transform/position/x",x,0.0);
    node.param("/transform/position/y",y,0.0);
    node.param("/transform/position/z",z,0.0);
    node.param("/transform/orientation/roll",roll,0.0);
    node.param("/transform/orientation/pitch",pitch,0.0);
    node.param("/transform/orientation/yaw",yaw,0.0);
    
    while (node.ok()){
//        static tf::TransformBroadcaster br1;
//        tf::Transform transform1;
//        transform1.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
//        tf::Quaternion q1;
//        q1.setRPY(0, 1.57, -1.57);
//        transform1.setRotation(q1);
//        br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "camera_link", "camera_depth_optical_frame"));
        
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(x, y, z) );
        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_link"));
        rate.sleep();
        }
    
    ros::spin();
    return 0;
};