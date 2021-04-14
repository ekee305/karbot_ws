
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

//test for branch

//Global variables
std::vector<int> map;
bool map_loaded_flag=1;

/*struct path_point {
  
  path_point(geometry_msgs::Point p) : point(p) {};
  
  geometry_msgs::Point point;
  int id;
  int parent_point_id;
  std::vector<Node> children;
  
}*/



void chatterCallback(const nav_msgs::OccupancyGrid &msg)
{
  ROS_INFO("number of elements %d", int(msg.data.size()));
  for(int i = 0; i <= msg.data.size(); i++) {
    map.push_back(msg.data[i]);
  }
  ROS_INFO("Map loaded");
  map_loaded_flag=0;
}

class RRT {
public:
	
	
private:
	double distance,goal_radius;
	int node_num;
	int resolution;
	std::vector<geometry_msgs::Point> children;
    std::vector<geometry_msgs::Point> parents;
    std::vector<geometry_msgs::Point> all_nodes;
	std::vector<geometry_msgs::Point> path;
	geometry_msgs::Point start, goal;
	
public:
	RRT(double d, double x_start, double y_start, int r,double x_goal, double y_goal,double goal_r) : distance (d), resolution(r),goal_radius(goal_r) {
		node_num=0;
		start.x=x_start;
		start.y=y_start;
		goal.x=x_goal;
		goal.y=y_goal;
		srand((unsigned int)time(NULL));
		all_nodes.push_back(start);
		
		
	}
	
	geometry_msgs::Point get_start() {
		return(start);
	}

	geometry_msgs::Point get_goal() {
		return(goal);
	}

	geometry_msgs::Point get_parents_value(int i) {
		return(parents[i]);
	}
	
	geometry_msgs::Point get_children_value(int i) {
		return(children[i]);
	}

	geometry_msgs::Point get_all_nodes_value(int i) {
		return(all_nodes[i]);
	}

	void set_goal (double x, double y) {
		goal.x=x;
		goal.y=y;
	}

	
	geometry_msgs::Point get_rand_point(){	
		geometry_msgs::Point point;
		point.x=static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10.0));
		point.y=static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10.0));
		//ROS_INFO(" rand point = (%lf, %lf)",point.x, point.y);
		return (point);
	}
	
	geometry_msgs::Point new_point(geometry_msgs::Point parent, geometry_msgs::Point random_point){
		double angle,x,y;
		x=random_point.x-parent.x;
		y=random_point.y-parent.y;
		geometry_msgs::Point child;
		angle=atan2(y,x);
		ROS_INFO("angle is %f",angle);
		child.x=parent.x+distance*cos(angle);
		child.y=parent.y+distance*sin(angle);
		//ROS_INFO("parent is (%f,%f), child is (%f,%f)",parent.x,parent.y,child.x, child.y); 
		return(child);
	}

	void push_nodes(geometry_msgs::Point parent, geometry_msgs::Point child){
		children.push_back(child);
		parents.push_back(parent);
		all_nodes.push_back(child);		
		node_num++; //used for printing parent and child node values for particular iteration, use node_num for referenceing all nodes and nod_num-1 for children or parent
	}


	int find_closest(geometry_msgs::Point random_point){
	int near=0;
	double temp=0;
	double mag=1000000;
	double x_diff,y_diff;
    for(int i = 0;i<all_nodes.size();i++){
    	x_diff=random_point.x-all_nodes[i].x;
    	y_diff=random_point.y-all_nodes[i].y;
    	temp=sqrt(pow(y_diff,2)+pow(x_diff,2));
    	if(temp<mag){
        	mag=temp;
        	near=i;
      	}    
    }
	//ROS_INFO("node is (%f,%f)",all_nodes[near].x,all_nodes[near].y);
	return (near);  
	}

	geometry_msgs::Point find_grid_cell(geometry_msgs::Point new_point) {
		geometry_msgs::Point grid;
		grid.x= new_point.x/resolution;
		grid.y= new_point.y/resolution;
		//ROS_INFO("grid is (%d,%d)",int(grid.x),int(grid.y));
		return(grid);
	}

	// to convert grid cell to array number for map
	int convert_grid_cell (geometry_msgs::Point grid) {
		int array=(int(grid.y)*10)+int(grid.x);
		return(array);
	}

	bool is_goal_found(){
		double x_diff,y_diff;
		x_diff=goal.x-all_nodes[node_num].x;
		y_diff=goal.y-all_nodes[node_num].y;
		double dist_to_goal=sqrt(pow(y_diff,2)+pow(x_diff,2));
		if(dist_to_goal<goal_radius){
			ROS_INFO("goal found (%lf,%lf)",children[node_num-1].x,children[node_num-1].y);
			ROS_INFO("goal found (%lf,%lf)",all_nodes[node_num].x,all_nodes[node_num].y);
			return(true);
		} else {
			return(false);
		}
	}

	bool at_start(geometry_msgs::Point test) {
		if (test == start){
			return (true);
		} else {
			return (false);
		}
	}

	void find_path() {
		geometry_msgs::Point temp_child;
		int index;
		path.push_back(children[node_num-1]);
		temp_child=parents[node_num-1];
		while(1) {
			index=find_child_index(temp_child);
			path.push_back(children[index]);
			if (at_start(parents[index])){
				path.push_back(parents[index]);
				return;
			} 
			temp_child=parents[index];
		}
		

	}

	int find_child_index(geometry_msgs::Point temp_child) {
		int i;
		for (i=0;i<children.size();i++){
			if (children[i].x == temp_child.x && children[i].y == temp_child.y){
				return(i);
			}
		}
		//print_path();
		ROS_INFO("Error in find_parent_index");
		while(1);
	}

	void print_path() {
		  for(int i = 0; i < path.size(); i++) {
    			ROS_INFO("path (%lf,%lf",path[i].x,path[i].y);
  			}
	}

	int path_length() {
		return(path.size());
	}

	geometry_msgs::Point get_path_point(int i) {
		return(path[i]);

	}
	

};
  


	
int main(int argc, char **argv)
{
  //ros setup
	static const int rate=1000;
	ros::init(argc, argv, "path");
	ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::Subscriber sub = nh.subscribe("map", 1000, chatterCallback);
	ros::Rate r(rate);
  
  //initialize RRT object and variables
	static const double child_distance=0.1;
	static const double x_start=5;
	static const double y_start=5; 
	static const double x_goal=9.5;
	static const double y_goal=9.5;
	static const int resolution=1; 
	static const double radius_goal=0.25;
	RRT path_planning(child_distance,x_start,y_start,resolution,x_goal,y_goal,radius_goal);  //would intialize path planner to have root at robot base
	geometry_msgs::Point next_point,parent,rand_point;
	int index_of_closest=0;
	int array_grid=0;
	  
  
  //marker setyp		
	visualization_msgs::Marker points, line_list, goal, path;
	points.header.frame_id=line_list.header.frame_id= goal.header.frame_id = path.header.frame_id = "/my_frame";
	points.header.stamp= line_list.header.stamp = goal.header.stamp = path.header.stamp = ros::Time::now();
	points.ns="nodes";
	goal.ns="goal_node";
	line_list.ns="edges";
	path.ns="path_nodes";
	points.action=line_list.action=goal.action = path.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w=line_list.pose.orientation.w=goal.pose.orientation.w= path.pose.orientation.w =1.0;

	points.id=0;
	line_list.id=1;
	goal.id=3;
	path.id=4;

	points.type = visualization_msgs::Marker::POINTS;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	goal.type = visualization_msgs::Marker::POINTS;
	path.type = visualization_msgs::Marker::POINTS;

	points.scale.x = 0.05;
	points.scale.y = 0.05;
	goal.scale.x = 0.1;
	goal.scale.y = 0.1;
	path.scale.x = 0.1;
	path.scale.y = 0.1;

	line_list.scale.x = 0.01;

	points.color.b = 1.0f;
	points.color.a = 1.0;
	goal.color.g = 1.0f;
	goal.color.a = 1.0;
	path.color.a= 1.0;
	path.color.r = 1.0f;

	line_list.color.r = 1.0;
	line_list.color.a = 1.0;

	//start postion for tree

	points.points.push_back(path_planning.get_start());
	goal.points.push_back(path_planning.get_goal());

  
  
	while(ros::ok()){
		
		ros::spinOnce();
		while (marker_pub.getNumSubscribers() < 1) {	
	  		if (!ros::ok())
	  			{
					return 0;
	  			}
	  		ROS_WARN_ONCE("Please create a subscriber to the marker");
			ros::spinOnce();
	  		sleep(1);
		}

		while (map_loaded_flag);

		//ROS_INFO("New Point");
		
		if (!path_planning.is_goal_found()){
			rand_point=path_planning.get_rand_point();
			//ROS_INFO("int is %d",find_closest(rand_point, all_nodes));


			index_of_closest=path_planning.find_closest(rand_point);
			parent=path_planning.get_all_nodes_value(index_of_closest);
			next_point=path_planning.new_point(parent,rand_point);
			array_grid=path_planning.convert_grid_cell(path_planning.find_grid_cell(next_point));
			//ROS_INFO("Array value is %d",array_grid);

			//if statement to implement obstacle avoidance, could make function in RRT class to do this;
			if (map[array_grid] < 70) {
				path_planning.push_nodes(parent,next_point);
				points.points.push_back(next_point);
				line_list.points.push_back(parent);
				line_list.points.push_back(next_point);
				marker_pub.publish(points);
				marker_pub.publish(goal);
				marker_pub.publish(line_list);
				//ROS_INFO("Point placement complete");
				//ROS_INFO(" ");
			} else {
				//ROS_INFO("Point in obstacle");
				//ROS_INFO(" ");
			}
		} else {
			path_planning.find_path();
			//path_planning.print_path();
			ROS_INFO("found and printed path");
			for (int i = path_planning.path_length()-1; i >=0 ;i--){
					path.points.push_back(path_planning.get_path_point(i));
			}
			marker_pub.publish(path);
			return(1);
		}

		//block loop
		//while(1)

		r.sleep();
	}   
}















































