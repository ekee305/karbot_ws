#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <gazebo_msgs/GetModelState.h>
#include <string.h>
#include <stdio.h>
#include <tf/tf.h>
#include<math.h>
#include <path_planning/path_to_goal.h>

#define MAX_SPEED 0.3
#define PI 3.14159265359

geometry_msgs::Point next_path_point;
geometry_msgs::Point position;
double roll,pitch, yaw;
geometry_msgs::Point goal;
bool goal_received=false;

bool path_loaded=false;
bool first_pose_loaded=false;
std::vecotr<


void pathCallback(const geometry_msgs::Point &msg) //might be quicker way of loading map by simply equating to data
{
	next_path_point=msg;
    ROS_INFO("Path loaded");
    path_loaded=true;
}

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) //might be quicker way of loading map by simply equating to data
{
	position.x=msg->pose.pose.position.x;
    position.y=msg->pose.pose.position.y;
    tf::Quaternion q( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    first_pose_loaded=true;

}

void goalCallback(const geometry_msgs::PoseStamped &msg) 
{
	if(goal.x != msg.pose.position.x && goal.y != msg.pose.position.y){
		goal.x=msg.pose.position.x;
    	goal.y=msg.pose.position.y;
		goal_received=true;
	} 
}


double get_dist(geometry_msgs:: Point point_1, geometry_msgs::Point point_2);







int main(int argc, char **argv)
{

    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n;
    ros::NodeHandle r;
    ros::NodeHandle a;
    ros::Subscriber path_sub = n.subscribe("/next_point_on_path", 1000, pathCallback);
    ros::Subscriber amcl_sub = a.subscribe("/amcl_pose", 1000, amclCallback);
    ros::Subscriber goal_sub = g.subscribe("/goal", 1, goalCallback);
    bool reached_goal = false;

    /*while(!path_loaded || !first_pose_loaded || !goal_received){
        ros::spinOnce();
    }
    ROS_INFO("path loaded");*/



    //intialise controller class
   //ROS_INFO("inital ref = (%lfm%lf)",path[path_end].x,path[path_end].y);

    
    ros::Rate loop_rate(1);
    //while (ros::ok())
    //{

        std::ofstream myfile;
        myfile.open ("example.csv");
        myfile << "This is the first cell in the first column.\n";
        myfile << "a,b,c,\n";
        myfile << "c,s,v,\n";
        myfile << "1,2,3.456\n";
        myfile << "semi;colon";
        myfile.close();
        return 0;
        

    //ros::spinOnce();
    //}


    return 0;
}


double get_dist(geometry_msgs:: Point point_1, geometry_msgs::Point point_2){
    double x_diff;
    double y_diff;
    double dist;
    x_diff=point_1.x-point_2.x;
    y_diff=point_1.y-point_2.y;
    return(sqrt(pow(y_diff,2)+pow(x_diff,2)));
}