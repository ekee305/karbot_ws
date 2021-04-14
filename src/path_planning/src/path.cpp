
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <array>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <stdlib.h> 
#include <ctime>
#include "std_msgs/String.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>


#define PI 3.14159265

/*struct path_point {
  
  path_point(geometry_msgs::Point p) : point(p) {};
  
  geometry_msgs::Point point;
  int id;
  int parent_point_id;
  std::vector<Node> children;
  
}

class RRT {
  public:
  
  
  
  
  
  
  
  
  private:
  std::vector<Node> tree;
  
}*/

geometry_msgs::Point get_rand_point();
geometry_msgs::Point new_point(geometry_msgs::Point parent, geometry_msgs::Point random_point, double distance, std::vector<geometry_msgs::Point> &children, std::vector<geometry_msgs::Point> &parents, int iterations);
int find_closest(geometry_msgs::Point random_point, std::vector<geometry_msgs::Point> &nodes);


	
int main(int argc, char **argv)
{
  //ros setup
  ros::init(argc, argv, "path");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Rate r(100);
  
  //seed rand()
  srand((unsigned int)time(NULL));
  
  //RRT variables
  double distance=0.5;
  int iterations=0;
  std::vector<geometry_msgs::Point> children;
  std::vector<geometry_msgs::Point> parents;
  std::vector<geometry_msgs::Point> all_nodes;
  
  
  //marker setyp
  visualization_msgs::Marker points, line_list;
  points.header.frame_id=line_list.header.frame_id= "/my_frame";
  points.header.stamp= line_list.header.stamp = ros::Time::now();
  points.ns="nodes";
  line_list.ns="edges";
  points.action=line_list.action=visualization_msgs::Marker::ADD;
  points.pose.orientation.w=line_list.pose.orientation.w=1.0;

  points.id=0;
  line_list.id=1;

  points.type = visualization_msgs::Marker::POINTS;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  points.scale.x = 0.05;
  points.scale.y = 0.05;
  
  line_list.scale.x = 0.01;

  points.color.g = 1.0f;
  points.color.a = 1.0;
  
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  
  //start postion for tree
  
  geometry_msgs::Point start,next_point,parent,rand_point;
  start.x=5;
  start.y=5;
  all_nodes.push_back(start);
  points.points.push_back(start);
 
  
  
  while(ros::ok()){
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    int index_of_closest=0;
    rand_point=get_rand_point();
    //ROS_INFO("int is %d",find_closest(rand_point, all_nodes));
    
    //add obstacle avoidance 
    
    index_of_closest=find_closest(rand_point, all_nodes);
    parent=all_nodes[index_of_closest];
    next_point=new_point(parent,rand_point,distance,children,parents,iterations);
    all_nodes.push_back(next_point);
    points.points.push_back(next_point);
    line_list.points.push_back(parent);
    line_list.points.push_back(next_point);

    marker_pub.publish(points);
    marker_pub.publish(line_list);
    iterations++;
    //block loop
    //while(1)
    r.sleep();
  }   
}


geometry_msgs::Point get_rand_point(){

  geometry_msgs::Point point;
  point.x=static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10.0));
  point.y=static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10.0));
  ROS_INFO("point x = %lf point y = %lf",point.x, point.y);
  return (point);
}

geometry_msgs::Point new_point(geometry_msgs::Point parent, geometry_msgs::Point random_point, double distance,std::vector<geometry_msgs::Point> &children, std::vector<geometry_msgs::Point> &parents, int iterations){
  double angle,x,y;
  x=random_point.x-parent.x;
  y=random_point.y-parent.y;
  geometry_msgs::Point child;
  angle=atan2(y,x);
  ROS_INFO("angle is %f",angle);
  child.x=parent.x+distance*cos(angle);
  child.y=parent.y+distance*sin(angle);
  children.push_back(child);
  parents.push_back(parent);
  ROS_INFO("parent is (%f,%f), child is (%f,%f)",parents[iterations].x,parents[iterations].y,children[iterations].x, children[iterations].y);
  return(child);
}


int find_closest(geometry_msgs::Point random_point, std::vector<geometry_msgs::Point> &nodes){
  int near=0;
  double temp=0;
  double mag=1000000;
  double x_diff,y_diff;
    for(int i = 0;i<nodes.size();i++){
      x_diff=random_point.x-nodes[i].x;
      y_diff=random_point.y-nodes[i].y;
      temp=sqrt(pow(y_diff,2)+pow(x_diff,2));
      if(temp<mag){
        mag=temp;
        near=i;
      }    
    }
  ROS_INFO("node is (%f,%f)",nodes[near].x,nodes[near].y);
  
  return (near);  
  
  
}







































