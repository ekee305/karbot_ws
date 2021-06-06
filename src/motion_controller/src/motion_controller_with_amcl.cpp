
//Authors: Ethan Kee
//Date:17/05/2021
//Description: Implementation of proportional controller for ROS


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <string.h>
#include <stdio.h>
#include<math.h>
#include <path_planning/path_to_goal.h>
#include "control.cpp"



geometry_msgs::Point next_path_point;
geometry_msgs::Point position;
double roll,pitch, yaw;

bool path_loaded=false;
bool first_pose_loaded=false;

//call back when path point recieved
void pathCallback(const geometry_msgs::Point &msg) //might be quicker way of loading map by simply equating to data
{
	next_path_point=msg;
    ROS_INFO("Path loaded");
    path_loaded=true;
}

// call back when AMCL postion is recieved
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) //might be quicker way of loading map by simply equating to data
{
	position.x=msg->pose.pose.position.x;
    position.y=msg->pose.pose.position.y;
    tf::Quaternion q( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    first_pose_loaded=true;

}
// a class to implmennt the control algorithm f the car




int main(int argc, char **argv)
{

    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n;
    ros::NodeHandle r;
    ros::NodeHandle a;
    geometry_msgs::Twist control_signal;
    //ros::Publisher control_pub = r.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 200);
    ros::Publisher control_pub = r.advertise<geometry_msgs::Twist>("/cmd_vel", 200);
    ros::Subscriber path_sub = n.subscribe("/next_point_on_path", 1000, pathCallback);
    ros::Subscriber amcl_sub = a.subscribe("/amcl_pose", 1000, amclCallback);
    bool reached_goal = false;

    //wait for first path and pose to be received before proceeding
    while(!path_loaded || !first_pose_loaded){
        ros::spinOnce();
    }
    ROS_INFO("path loaded");

    control car(yaw,position.x,position.y,next_path_point.x,next_path_point.y);
    car.p_control();
    
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ROS_INFO("point=(%lf,%lf,%lf)",position.x,position.y,yaw);
        car.update_variables(position.x,position.y,yaw,next_path_point);
        //If dummy point is received stop the robot
        if (next_path_point.x == -1000) {
            control_signal.linear.x=0.000000;
            control_signal.angular.z=0.000000;
            control_pub.publish(control_signal);
        } else {
            car.p_control();
            control_signal.linear.x=car.get_velocity();
            control_signal.angular.z=car.get_steer_velocity();
            control_pub.publish(control_signal);
        }
        ros::spinOnce();
    }


    return 0;
}
