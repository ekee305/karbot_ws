#ifndef MY_CLASS_CONTROL
#define MY_CLASS_CONTROL

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <string.h>
#include <stdio.h>
#include<math.h>



class control
{
private:
    double x_pos,y_pos,z_angle,x_ref,y_ref,dist,heading_error;
public:
    double velocity,steer_velocity; // control velocity and steering angle to be published
public:
    control(double theta, double x1, double y1, double reference_x, double reference_y );

    //function to recalculate errors
    void update_errors();

    //function to update variables in object with ones passed
    void update_variables(double x1,double y1,double theta,geometry_msgs::Point next_point);
    //function to implement p control for car
    void p_control();

    double get_velocity();

    double get_steer_velocity();

    double get_heading_error();

    double get_dist_error();

    double get_dist_to_goal(geometry_msgs::Point goal);
    
};

#endif
