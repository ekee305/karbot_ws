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


double get_dist(geometry_msgs:: Point point_1, geometry_msgs::Point point_2);







int main(int argc, char **argv)
{

    ros::init(argc, argv, "path_length_reader");
    ros::NodeHandle n;
    ros::NodeHandle t;
    ros::NodeHandle g;
    ros::Subscriber path_sub = n.subscribe("/next_point_on_path", 1, pathCallback);
    ros::Subscriber time_sub = t.subscribe("/time", 1, timeCallback);
    ros::Subscriber goal_sub = g.subscribe("/goal", 1, goalCallback);

    geometry_msgs::Point old_goal,goal1,goal2,goal3,goal4;
    int i=0;
    bool first_run=true;

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

    double cost_to_goal=0;



    //intialise controller class
   //ROS_INFO("inital ref = (%lfm%lf)",path[path_end].x,path[path_end].y);
    std::ofstream myfile;
    ros::Rate loop_rate(100);
    while (ros::ok())
    {

       
       if (goal_changed && old_goal == goal1){
            myfile.open ("RT_RRT_path_data_goal_1.csv", std::ios_base::app);
            if(first_run) {
                myfile << "building tree \n";
                first_run=false;
            }
            for(int i=0;i < path_points.size()-2; i++){
                cost_to_goal=cost_to_goal+get_dist(path_points[i],path_points[i+1]);
            }
            myfile << cost_to_goal << "," << time_to_find_path.data << ",\n";
            path_points.clear();
            cost_to_goal=0;
            goal_changed=false;
            old_goal=goal;
            myfile.close();
        }  else if (goal_changed && old_goal == goal2){
            myfile.open ("RT_RRT_path_data_goal_2.csv", std::ios_base::app);
             for(int i=0;i < path_points.size()-2; i++){
                cost_to_goal=cost_to_goal+get_dist(path_points[i],path_points[i+1]);
            }
            myfile << cost_to_goal << "," << time_to_find_path.data << ",\n";
            path_points.clear();
            cost_to_goal=0;
            goal_changed=false;
            old_goal=goal;
            myfile.close();
        } else if (goal_changed && old_goal == goal3){
            myfile.open ("RT_RRT_path_data_goal_3.csv", std::ios_base::app);
            for(int i=0;i < path_points.size()-2; i++){
                cost_to_goal=cost_to_goal+get_dist(path_points[i],path_points[i+1]);
            }
            myfile << cost_to_goal << "," << time_to_find_path.data << ",\n";
            path_points.clear();
            cost_to_goal=0;
            goal_changed=false;
            old_goal=goal;
            myfile.close();
        } else if (goal_changed && old_goal == goal4){
            myfile.open ("RT_RRT_path_data_goal_4.csv", std::ios_base::app);
            for(int i=0;i < path_points.size()-2; i++){
                cost_to_goal=cost_to_goal+get_dist(path_points[i],path_points[i+1]);
            }
            myfile << cost_to_goal << "," << time_to_find_path.data << ",\n";
            path_points.clear();
            cost_to_goal=0;
            goal_changed=false;
            old_goal=goal;
            myfile.close();
            ROS_WARN("run %d",i++);
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