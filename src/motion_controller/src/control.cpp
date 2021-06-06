#include"motion_controller/control.h"
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

#define MAX_SPEED 0.6
#define MAX_ANGULAR_SPEED 3.1
#define PI 3.14159265359
    
    
control::control(double theta, double x1, double y1, double reference_x, double reference_y ) : z_angle(theta), x_pos(x1), y_pos(y1), x_ref(reference_x),y_ref(reference_y)
{}

//function to recalculate errors
void control::update_errors()
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
void control::update_variables(double x1,double y1,double theta,geometry_msgs::Point next_point){
    x_pos=x1;
    y_pos=y1;
    z_angle=theta;
    x_ref=next_point.x;
    y_ref=next_point.y;
}
//function to implement p control for car
void control::p_control()
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

double control::get_velocity()
{
    return(velocity);
}


double control::get_steer_velocity()
{
    return(steer_velocity);
}

double control::get_heading_error(){
    return(heading_error);
}

double control::get_dist_error(){
    double x_dif,y_dif;
    x_dif=x_ref-x_pos;
    y_dif=y_ref-y_pos;
    return(sqrt((x_dif*x_dif)+(y_dif*y_dif)));
}

double control::get_dist_to_goal(geometry_msgs::Point goal){
    double x_dif,y_dif;
    x_dif=goal.x-x_pos;
    y_dif=goal.y-y_pos;
    dist=sqrt((x_dif*x_dif)+(y_dif*y_dif));
}
    
