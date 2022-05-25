#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "camera_tf_broadcaster");

    ros::NodeHandle node;
    
    while (ros::ok){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 1.45) );
        tf::Quaternion q;
        q.setRPY(0, 0.279253, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_link"));
        }

    ros::spin();
    return 0;
};