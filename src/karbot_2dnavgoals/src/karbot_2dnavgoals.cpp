#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <actionlib/client/simple_action_client.h>
#include <sstream>

double waitingRoom_x = 11.4
double waitingRoom_y = 13.1

int main(int argc, char **argv) {

ros::init(argc, argv, "2d_nav_goal");

ros::NodeHandle n;

ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal, 1000);

ros::Rate loop_rate(1000);

int count=0;
while (ros::ok()) {

geometry_msgs::PoseStamped goal;

   goal.header.frame_id="map";

   goal.pose.position.x=waitingRoom_x;
   goal.pose.position.y=waitingRoom_y;
   goal.pose.position.z = 0.0;

   goal.pose.orientation.x = 0.0;
   goal.pose.orientation.y = 0.0;
   goal.pose.orientation.z = -0.7;
   goal.pose.orientation.w = 0.7;

   ROS_INFO("%lf",goal.pose.position.x);

   pub.publish(goal);

   ros::spinOnce();

   loop_rate.sleep();

   }


  return 0;
}









