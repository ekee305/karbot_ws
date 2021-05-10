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
std_msgs::Float64 time_to_find_path,cost_to_goal;

std::vector<geometry_msgs::Point> path_points;




void costCallback(const std_msgs::Float64 &msg) //might be quicker way of loading map by simply equating to data
{
    cost_to_goal=msg;
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
        goal_changed=true;
	} 
}










int main(int argc, char **argv)
{

    ros::init(argc, argv, "path_length_reader");
    ros::NodeHandle n;
    ros::NodeHandle t;
    ros::NodeHandle g;
    ros::Subscriber cost_sub = n.subscribe("/cost", 1, costCallback);
    ros::Subscriber time_sub = t.subscribe("/time", 1, timeCallback);
    ros::Subscriber goal_sub = g.subscribe("/goal", 1, goalCallback);

    geometry_msgs::Point old_goal,goal1,goal2,goal3,goal4;
    int i=0;
    
    goal1.x=21;
    goal1.y=20.5;
    
    goal2.x=28.3;
    goal2.y=24.06;
    
    goal3.x=28.4;
    goal3.y=29;
    
    goal4.x=25;
    goal4.y=25;




    while(!goal_received){
       ros::spinOnce();
       
    }
    old_goal=goal;
    goal_changed=false;



    //intialise controller class
   //ROS_INFO("inital ref = (%lfm%lf)",path[path_end].x,path[path_end].y);
    std::ofstream myfile;
    ros::Rate loop_rate(100);
    while (ros::ok())
    {  
       if (goal_changed && old_goal == goal1){
           ROS_WARN("should be writing to file");
            myfile.open ("RRT_path_data_goal_1.csv", std::ios_base::app);
            myfile << cost_to_goal.data << "," << time_to_find_path.data << ",\n";
            goal_changed=false;
            old_goal=goal;
            myfile.close();
        }  else if (goal_changed && old_goal == goal2){
            myfile.open ("RRT_path_data_goal_2.csv", std::ios_base::app);
            myfile << cost_to_goal.data << "," << time_to_find_path.data << ",\n";
            goal_changed=false;
            old_goal=goal;
            myfile.close();
        } else if (goal_changed && old_goal == goal3){
            myfile.open ("RRT_path_data_goal_3.csv", std::ios_base::app);
            myfile << cost_to_goal.data << "," << time_to_find_path.data << ",\n";
            goal_changed=false;
            old_goal=goal;
            myfile.close();
        } else if (goal_changed && old_goal == goal4){
            myfile.open ("RRT_path_data_goal_4.csv", std::ios_base::app);
            myfile << cost_to_goal.data << "," << time_to_find_path.data << ",\n";
            goal_changed=false;
            old_goal=goal;
            myfile.close();
            ROS_WARN("run %d",i++);
        }
        ros::spinOnce();
    }



}


