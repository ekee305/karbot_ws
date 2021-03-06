#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_broadcaster");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(10.0);
    while (node.ok()){
        transform.setOrigin( tf::Vector3(-5, -5, 0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1.0) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
        rate.sleep();
    }
    return 0;
};