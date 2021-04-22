#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <actionlib/client/simple_action_client.h>
#include <sstream>
#include <geometry_msgs/Point.h>


int main(int argc, char **argv) {

  ros::init(argc, argv, "karbot_2dnavgoals");
  ros::NodeHandle n;
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/goal", 1000);
  ros::Rate loop_rate(1);

  while (ros::ok()) {

    geometry_msgs::PoseStamped goal;

    goal.header.frame_id="map";

    goal.pose.position.x=14.8;
    goal.pose.position.y=11.2;
    ROS_INFO("%lf",goal.pose.position.x);

    goal_pub.publish(goal);
    loop_rate.sleep();

  }
  return(0);  
}
  









