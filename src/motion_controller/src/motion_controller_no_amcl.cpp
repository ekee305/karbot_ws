#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include <gazebo_msgs/GetModelState.h>
#include <string.h>
#include <stdio.h>
#include <tf/tf.h>
#include<math.h>
#include <path_planning/path_to_goal.h>

#define PI 3.14159265359

std::vector<geometry_msgs::Point> path;
bool path_loaded=false;
int path_end;


void pathCallback(const path_planning::path_to_goal::ConstPtr& msg ) //might be quicker way of loading map by simply equating to data
{
	path=msg->path;
    ROS_INFO("Path loaded");
    path_loaded=true;
    path_end=path.size()-1;
}

class control
{
private:
    double x_pos,y_pos,z_angle,x_ref,y_ref,dist,heading_error;
public:
    double velocity,steer_velocity; //voltages to control velocity and steering angle
    int path_index;
public:
    control(double theta, double x1, double y1, double reference_x, double reference_y ) : z_angle(theta), x_pos(x1), y_pos(y1), x_ref(reference_x),y_ref(reference_y)
    {
        path_index=path_end;
    }

    /*void update_goal(geometry_msgs::Point goal){
    }*/

    void update_errors()
    {
        double x_dif,y_dif,square;
        x_dif=x_ref-x_pos;
        y_dif=y_ref-y_pos;
        dist=sqrt((x_dif*x_dif)+(y_dif*y_dif));
        heading_error=atan2(y_dif,x_dif)-z_angle;
        if (heading_error < -PI){
            heading_error=heading_error+2*PI;
        }
        if (heading_error > PI){
            heading_error=heading_error-2*PI;
        }
    }

    void update_variables(double x1,double y1,double theta){
        x_pos=x1;
        y_pos=y1;
        z_angle=theta;
    }

    void p_control()
    {
        double kps,kpv;
        kps=1.8;
        kpv=1.8;
        update_errors();
        steer_velocity=kps*heading_error;
        velocity=kpv*dist;
        if (velocity> 0.6){
            velocity=0.6;
        }
        /*if (steer_velocity < 0.5){
            steer_velocity=0.5;
        }*/
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
        ROS_INFO("refs(%lf,%lf",x_ref,y_ref);
        return(sqrt((x_dif*x_dif)+(y_dif*y_dif)));
    }

    double get_dist_to_goal(geometry_msgs::Point goal){
        double x_dif,y_dif;
        x_dif=goal.x-x_pos;
        y_dif=goal.y-y_pos;
        dist=sqrt((x_dif*x_dif)+(y_dif*y_dif));
    }
    
    void iterate_reference(){
        path_index=path_index-1;
        x_ref=path[path_index].x;
        y_ref=path[path_index].y;
    }
};







int main(int argc, char **argv)
{

    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n;
    ros::NodeHandle r;
    ros::Duration half_sec(0.5);
    bool result;
    geometry_msgs::Twist control_signal;
    ros::Publisher control_pub = r.advertise<geometry_msgs::Twist>("/cmd_vel", 200);
    ros::Subscriber sub = n.subscribe("/path", 1000, pathCallback);
    while(!path_loaded){
        ros::spinOnce();
    }
    ROS_INFO("path loaded");
    geometry_msgs::Point goal_point;
    goal_point.x=path[0].x;
    goal_point.y=path[0].y;
    /*goal_point.x=5;
    goal_point.y=5;*/


    bool service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/get_model_state",true);
      ROS_INFO("waiting for get_model_state service");
      half_sec.sleep();
    }
    
    //intialise server client
    ros::ServiceClient get_model_state_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    gazebo_msgs::GetModelState mybot_state;
    mybot_state.request.model_name = "karbot";
    mybot_state.request.relative_entity_name = "world";
    get_model_state_client.call(mybot_state);
    double roll,pitch, yaw;
    double x,y,theta;
    x=mybot_state.response.pose.position.x;
    y=mybot_state.response.pose.position.y;
    tf::Quaternion q( mybot_state.response.pose.orientation.x, mybot_state.response.pose.orientation.y, mybot_state.response.pose.orientation.z, mybot_state.response.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    theta=yaw;

    //intialise controller class
   //ROS_INFO("inital ref = (%lfm%lf)",path[path_end].x,path[path_end].y);
   control car(theta,x,y,path[path_end].x,path[path_end].y);
   //control car(theta,x,y,5,5);
    car.p_control();
    
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        get_model_state_client.call(mybot_state);
        result=mybot_state.response.success;
        if (!result){
            ROS_WARN("service call to get_model_state failed!");
        } else if (car.get_dist_to_goal(goal_point)<0.2) {
            ROS_INFO("car at goal");
            control_signal.linear.x=0.000000;
            control_signal.angular.z=0.000000;
            control_pub.publish(control_signal);
            return(1);
        } else {
            
            x=mybot_state.response.pose.position.x;
            y=mybot_state.response.pose.position.y;
            tf::Quaternion q( mybot_state.response.pose.orientation.x, mybot_state.response.pose.orientation.y, mybot_state.response.pose.orientation.z, mybot_state.response.pose.orientation.w);
            double roll,pitch, yaw;
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            theta=yaw;//theta seems to change
            if (car.get_dist_error() < 0.2){
                car.iterate_reference();
            }
            ROS_INFO("point is (%lf,%lf)",path[car.path_index].x,path[car.path_index].y);
            ROS_INFO("dist error=%lf",car.get_dist_error());
            car.update_variables(x,y,theta);
            car.p_control();
            control_signal.linear.x=car.get_velocity();
            control_signal.angular.z=car.get_steer_velocity();
            control_pub.publish(control_signal);
            //ROS_INFO("x=%lf,y=%lf, theta=%lf \n",x,y,theta);
            //ROS_INFO("vel=%lf steer=%lf",car.get_velocity(),car.get_steer_velocity());
        }
        ros::spinOnce();
    }


    return 0;
}
