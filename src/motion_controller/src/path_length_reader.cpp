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
#include <iostream>
#include <fstream>
#include "std_msgs/Float64.h"

#define MAX_SPEED 0.3
#define PI 3.14159265359

geometry_msgs::Point next_path_point;
geometry_msgs::Point position;
double roll,pitch, yaw;
geometry_msgs::Point goal;
bool goal_received=false;
bool goal_changed=false;
bool time_received=false;
std_msgs::Float64 time_to_find_path;

std::vector<geometry_msgs::Point> path_points;




void pathCallback(const geometry_msgs::Point &msg) //might be quicker way of loading map by simply equating to data
{
	if(msg != next_path_point){
        next_path_point=msg;
        if(next_path_point.x != -1000){
            path_points.push_back(next_path_point);
        }
    }
}

void timeCallback(const std_msgs::Float64 &msg) //might be quicker way of loading map by simply equating to data
{
    ROS_WARN("time callback");
    time_to_find_path=msg;
}


void goalCallback(const geometry_msgs::PoseStamped &msg) 
{
	if(goal.x != msg.pose.position.x && goal.y != msg.pose.position.y){
		goal.x=msg.pose.position.x;
    	goal.y=msg.pose.position.y;
		goal_received=true;
        if(!path_points.empty()){
            goal_changed=true;
        }
	} 
}


//double get_dist(geometry_msgs:: Point point_1, geometry_msgs::Point point_2);







int main(int argc, char **argv)
{

    ros::init(argc, argv, "path_length_reader");
    ros::NodeHandle n;
    ros::NodeHandle t;
    ros::NodeHandle g;
    ros::Subscriber path_sub = n.subscribe("/next_point_on_path", 1, pathCallback);
    ros::Subscriber time_sub = t.subscribe("/time", 1, timeCallback);
    ros::Subscriber goal_sub = g.subscribe("/goal", 1, goalCallback);

    geometry_msgs::Point old_goal;

    while(!goal_received){
       ros::spinOnce();
       
    }
    old_goal=goal;



    //intialise controller class
   //ROS_INFO("inital ref = (%lfm%lf)",path[path_end].x,path[path_end].y);
    std::ofstream myfile;
    myfile.open ("path_data.csv", std::ios_base::app);
    ros::Rate loop_rate(100);
    while (ros::ok())
    {

       
       if (goal_changed){
            myfile << "Path way points to (" << old_goal.x << " " << old_goal.y << ").\n";
            myfile << "x,y,\n";
            for (int i = 0; i < path_points.size(); i++){
                myfile << path_points[i].x << "," << path_points[i].y << ",\n";
            }
            myfile << "Time," << time_to_find_path.data << ",\n"; 
            path_points.clear();
            goal_changed=false;
            old_goal=goal;
        }
        

        
        
        ros::spinOnce();
    }



}


double get_dist(geometry_msgs:: Point point_1, geometry_msgs::Point point_2){
    double x_diff;
    double y_diff;
    double dist;
    x_diff=point_1.x-point_2.x;
    y_diff=point_1.y-point_2.y;
    return(sqrt(pow(y_diff,2)+pow(x_diff,2)));
}