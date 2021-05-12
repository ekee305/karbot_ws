#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <actionlib/client/simple_action_client.h>
#include <sstream>
#include <geometry_msgs/Point.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>

geometry_msgs::Point position;
double roll,pitch, yaw;
bool first_pose_loaded=false;

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) //might be quicker way of loading map by simply equating to data
{
	position.x=msg->pose.pose.position.x;
  position.y=msg->pose.pose.position.y;
  tf::Quaternion q( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  first_pose_loaded=true;

}

double get_dist(geometry_msgs:: Point point_1, geometry_msgs::Point point_2);

int main(int argc, char **argv) {

  ros::init(argc, argv, "karbot_2dnavgoals");
  ros::NodeHandle n;
  ros::NodeHandle a;
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/goal", 1000);
  ros::Subscriber amcl_sub = a.subscribe("/amcl_pose", 1, amclCallback);
  ros::Rate loop_rate(10);
  int index=0;

  while(!first_pose_loaded){
    ROS_WARN_ONCE("Waiting for AMCL");
    ros::spinOnce();
  }
  ROS_WARN_ONCE("AMCL received");
  std::deque<geometry_msgs::PoseStamped> goals;
  geometry_msgs::PoseStamped temp_goal;
  double goal_array[]={21,20.5,28.3,24.06,28.4,29,25,25};
  temp_goal.header.frame_id="map";
  for (int i = 0; i<8;i=i+2){
    temp_goal.pose.position.x=goal_array[i];
    temp_goal.pose.position.y=goal_array[i+1];
    goals.push_back(temp_goal);
  }

  while (ros::ok()) {
        goal_pub.publish(goals[index]);
        if ((get_dist(goals[index].pose.position,position))<0.5){
          index=((index+1) % 4);
        } 
   
    
    
    ros::spinOnce();
    loop_rate.sleep();

  }
  return(0);  
}
  


	double get_dist(geometry_msgs:: Point point_1, geometry_msgs::Point point_2){
		double x_diff;
		double y_diff;
		double dist;
    	x_diff=point_1.x-point_2.x;
    	y_diff=point_1.y-point_2.y;
		return(sqrt(pow(y_diff,2)+pow(x_diff,2)));
	}









