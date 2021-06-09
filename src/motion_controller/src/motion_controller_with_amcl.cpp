//Authors: Ethan Kee
//Date:17/05/2021
//Description: Implementation of proportional controller for ROS


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

#define MAX_SPEED 0.2
#define MAX_ANGULAR_SPEED 1.5
#define PI 3.14159265359

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
class control
{
private:
    double x_pos,y_pos,z_angle,x_ref,y_ref,dist,heading_error;
public:
    double velocity,steer_velocity; // control velocity and steering angle to be published
public:
    control(double theta, double x1, double y1, double reference_x, double reference_y ) : z_angle(theta), x_pos(x1), y_pos(y1), x_ref(reference_x),y_ref(reference_y)
    {}

    //function to recalculate errors
    void update_errors()
    {
        double x_dif,y_dif,square;
        x_dif=x_ref-x_pos;
        y_dif=y_ref-y_pos;
        dist=sqrt((x_dif*x_dif)+(y_dif*y_dif));
        heading_error=atan2(y_dif,x_dif)-z_angle;
        //required to wrap heading error between PI and -PI
        if (heading_error < -PI){
            heading_error=heading_error+2*PI;
        }
        if (heading_error > PI){
            heading_error=heading_error-2*PI;
        }
    }
    //function to update variables in object with ones passed
    void update_variables(double x1,double y1,double theta,geometry_msgs::Point next_point){
        x_pos=x1;
        y_pos=y1;
        z_angle=theta;
        x_ref=next_point.x;
        y_ref=next_point.y;
    }
    //function to implement p control for car
    void p_control()
    {
        double kps,kpv;
        kps=2.5;
        kpv=1.8;
        update_errors();
        steer_velocity=kps*heading_error;
        velocity=kpv*dist;
        if (velocity> MAX_SPEED){
            velocity=MAX_SPEED;
        }
        if (steer_velocity > MAX_ANGULAR_SPEED){
            steer_velocity=MAX_ANGULAR_SPEED;
        } else if (steer_velocity < -MAX_ANGULAR_SPEED) {
            steer_velocity=-MAX_ANGULAR_SPEED;
        }
        // preventing forward movment while heading error large
        if (heading_error > PI/8 || heading_error <-PI/8){
            velocity=0.0;
        }

    }

    double get_velocity()
    {
        return(velocity);
    }


    double get_steer_velocity()
    {
        return(steer_velocity);
    }

    double get_heading_error(){
        return(heading_error);
    }

    double get_dist_error(){
        double x_dif,y_dif;
        x_dif=x_ref-x_pos;
        y_dif=y_ref-y_pos;
        return(sqrt((x_dif*x_dif)+(y_dif*y_dif)));
    }

    double get_dist_to_goal(geometry_msgs::Point goal){
        double x_dif,y_dif;
        x_dif=goal.x-x_pos;
        y_dif=goal.y-y_pos;
        dist=sqrt((x_dif*x_dif)+(y_dif*y_dif));
    }
    
};







int main(int argc, char **argv)
{

    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n;
    ros::NodeHandle r;
    ros::NodeHandle a;
    geometry_msgs::Twist control_signal;
    ros::Publisher control_pub = r.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 200);
    //ros::Publisher control_pub = r.advertise<geometry_msgs::Twist>("/cmd_vel", 200);
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
