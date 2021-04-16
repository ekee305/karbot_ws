
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
#include <path_planning/grid_cell.h>
#include <random>




#define PI 3.14159265
#define OBSTACLE_THRESHOLD 20.0

//test for branch
//lol

//Global variables
std::vector<int> map;
double resolution_from_map;
bool map_loaded_flag=1;
int map_width,map_height;//should change to local variable;
std::default_random_engine re;


void chatterCallback(const nav_msgs::OccupancyGrid &msg) //might be quicker way of loading map by simply equating to data
{
	ROS_INFO("number of elements %d", int(msg.data.size()));
	map.insert(map.end(), &msg.data[0], &msg.data[msg.data.size()]);
	ROS_INFO("Map loaded");
	resolution_from_map=msg.info.resolution;
	map_width=msg.info.width;
	map_height=msg.info.height;
	map_loaded_flag=0;
}

class RRT {
public:
	
	
private:
	double distance,goal_radius,neighbour_radius;
	double resolution; // get this from the map data coming from the server.

    std::vector<geometry_msgs::Point> all_nodes;
	std::vector<geometry_msgs::Point> path;
	geometry_msgs::Point start, goal;

	struct node 
	{
		int ID;
		geometry_msgs::Point point; 
		double cost;
		node* parent;
		std::vector<node*> children;
	};	

	node* root;
	std::vector<node*> node_list;
	int I;


public:
	RRT(double d, double x_start, double y_start, double r,double x_goal, double y_goal,double goal_r,double neighbour_r) : distance (d), resolution(r),goal_radius(goal_r), neighbour_radius(neighbour_r) {
		start.x=x_start;
		start.y=y_start;		
		goal.x=x_goal;
		goal.y=y_goal;
		srand((unsigned int)time(NULL));
		I=0;
		create_root();

		
		
	}

	void create_root() 
	{
		node* new_node= new node;
		new_node->ID = I++;
		new_node->parent = NULL;
		new_node->cost = 0;
		new_node->point.x=start.x;
		new_node->point.y=start.y;
		all_nodes.push_back(start);
		node_list.push_back(new_node);
		root = new_node;
	}

	void add_node_to_tree(int index, geometry_msgs::Point new_point) 
	{
	
		node* new_node = new node;
		new_node->ID = I++;
		new_node->point.x=new_point.x;
		new_node->point.y=new_point.y;
		new_node->parent = node_list[index]; //use postion of parent in all_nodes to find pointer to the 
		new_node->cost = calculate_cost(node_list[index],new_node);
		update_parent(index, new_node);
		all_nodes.push_back(new_point);
		node_list.push_back(new_node);
	}

	double calculate_cost (node* parent, node* child)
	{
		double x_diff;
		double y_diff;
		double cost;
    	x_diff=parent->point.x-child->point.x;
    	y_diff=parent->point.y-child->point.y;
    	cost=parent->cost+sqrt(pow(y_diff,2)+pow(x_diff,2));
	}

	double get_cost(int index)
	{
		return(node_list[index]->cost);
	}

	void update_parent(int index, node *child) 
	{
		node_list[index]->children.push_back(child);
	}
	
	geometry_msgs::Point get_start() {
		return(start);
	}

	geometry_msgs::Point get_goal() {
		return(goal);
	}
	int get_I () {
		return(I);
	}

	geometry_msgs::Point get_all_nodes_value(int i) {
		return(all_nodes[i]);
	}


	void set_goal (double x,double y) {
		goal.x=x;
		goal.y=y;
	}
	
	geometry_msgs::Point get_rand_point(std::uniform_real_distribution<double> unif_x, std::uniform_real_distribution<double> unif_y){	
		geometry_msgs::Point point;
		point.x=unif_x(re);
		point.y=unif_y(re);
		//ROS_INFO(" rand point = (%lf, %lf)",point.x, point.y);
		return (point);
	}
	
 	geometry_msgs::Point new_point(geometry_msgs::Point closest, geometry_msgs::Point random_point){
		double angle,x,y;
		x=random_point.x-closest.x;
		y=random_point.y-closest.y;
		geometry_msgs::Point next_point;
		angle=atan2(y,x);
		next_point.x=closest.x+distance*cos(angle);
		next_point.y=closest.y+distance*sin(angle);
		//ROS_INFO("parent is (%f,%f), child is (%f,%f)",parent.x,parent.y,child.x, child.y); 
		return(next_point);
	}

	int check_neighbours (geometry_msgs::Point new_point,int index_of_closest){
		std::vector<node*> neighbours;
		double cost_from_neighbour,total_cost;
		double lowest_cost=1000000;
		double x_diff,y_diff;
		int closest_neighbour_index=-1;
		for(int i = 0;i<all_nodes.size();i++){
			x_diff=new_point.x-all_nodes[i].x;
			y_diff=new_point.y-all_nodes[i].y;
			cost_from_neighbour=sqrt(pow(y_diff,2)+pow(x_diff,2));
			if(cost_from_neighbour <= neighbour_radius && !check_line_obstacle(new_point,all_nodes[i])){
				neighbours.push_back(node_list[i]);
			}    
		}
		for (int i = 0;i<neighbours.size();i++){
			x_diff=new_point.x-neighbours[i]->point.x;
			y_diff=new_point.y-neighbours[i]->point.y;
			cost_from_neighbour=sqrt(pow(y_diff,2)+pow(x_diff,2));
			total_cost=neighbours[i]->cost+cost_from_neighbour;
			if (total_cost < lowest_cost){
				closest_neighbour_index=neighbours[i]->ID;
				lowest_cost=total_cost;
	
			}
		}
		return(closest_neighbour_index);
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
		return (near);  
	}
	
	path_planning::grid_cell find_grid_cell(geometry_msgs::Point new_point) {
		path_planning::grid_cell grid;
		grid.x= int(new_point.x/resolution); //replace with reolution that is given with occupancy grid data.
		grid.y= int(new_point.y/resolution);
		//ROS_INFO("grid is (%d,%d)",grid.x,grid.y);
		return(grid);
	}

	int convert_grid_cell (path_planning::grid_cell grid) {
		int map_array_value=grid.y*map_width*resolution+grid.x;
		return(map_array_value);
	}

	int round_to_grid_cell (double coord){
		return(int(coord/resolution));
	}

	bool is_goal_found(){
		double x_diff,y_diff;
		x_diff=goal.x-all_nodes[I-1].x;
		y_diff=goal.y-all_nodes[I-1].y;
		double dist_to_goal=sqrt(pow(y_diff,2)+pow(x_diff,2));
		if(dist_to_goal<goal_radius){
			ROS_INFO("goal found");
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
		geometry_msgs::Point temp_point;
		node* node_ptr= node_list[I-1];
		temp_point.x=node_ptr->point.x;
		temp_point.y=node_ptr->point.y;
		path.push_back(temp_point);
		while(temp_point != start) 
		{
			node_ptr=get_parent(node_ptr);
			temp_point.x=node_ptr->point.x;
			temp_point.y=node_ptr->point.y;
			path.push_back(temp_point);
		}
		

	}

	void print_path() {
		for(int i = 0; i < path.size(); i++) {
			ROS_INFO("path (%lf,%lf",path[i].x,path[i].y);
		}
	}

	node* get_parent(node* child)
	{
		return(child->parent);
	}

	int path_length() {
		return(path.size());
	}

	geometry_msgs::Point get_path_point(int i) {
		return(path[i]);
	}

	bool check_line_obstacle (geometry_msgs::Point point_1, geometry_msgs::Point point_2) {
    geometry_msgs::Point test_point_1,test_point_2;
    path_planning::grid_cell grid_1,grid_2,grid_1_test,grid_2_test;
		int difference_x,difference_y;
		int map_test;
    double grad, c,x_test,y_test,angle;
    if(point_1.x > point_2.x){
      test_point_1=point_2;
      test_point_2=point_1;
    }else {
      test_point_1=point_1;
      test_point_2=point_2;
    }
    grad=(test_point_2.y-test_point_1.y)/(test_point_2.x-test_point_1.x);
    c=(test_point_2.y-grad*test_point_2.x);
    angle=atan(grad);

		grid_1=find_grid_cell(test_point_1); 
		grid_2=find_grid_cell(test_point_2);
		difference_x=grid_2.x-grid_1.x;
    difference_y=grid_2.y-grid_1.y;
    if (test_point_1.x==test_point_2.x){
      if (difference_y >= 0) {
        for(int i = grid_1.y+1; i < grid_1.y+difference_y+1; i++) {
          grid_2_test.x=grid_2.x;
          grid_1_test.x=grid_1.x;
          grid_2_test.y=i;
          grid_1_test.y=i-1;
          if(check_grid_for_obs(grid_1_test)) {
            return(true);
          }
          if(check_grid_for_obs(grid_2_test)) {
            return(true);
          }
        }
      } else if (difference_y < 0){
        for(int i = grid_1.y; i > grid_2.y; i--) {
          grid_2_test.x=grid_2.x;
          grid_1_test.x=grid_1.x;
          grid_2_test.y=i;
          grid_1_test.y=i-1;
          if(check_grid_for_obs(grid_1_test)) {
            return(true);
          }
          if(check_grid_for_obs(grid_2_test)) {
            return(true);
          }
        }
      }
    } else if (difference_y >= 0) {
      if (angle < PI/4){
        for(int i = grid_1.x+1; i < grid_1.x+difference_x+1; i++) {
          y_test=(grad*i*resolution)+c;
          grid_2_test.y=int(y_test/resolution);
          grid_1_test.y=int(y_test/resolution);
          grid_2_test.x=i;
          grid_1_test.x=i-1;
          if(check_grid_for_obs(grid_1_test)) {
            return(true);
          }
          if(check_grid_for_obs(grid_2_test)) {
            return(true);
          }
        }
      } else {
        for(int i = grid_1.y+1; i < grid_1.y+difference_y+1; i++) {
          x_test=((i*resolution)-c)/grad;
          grid_2_test.x=int(x_test/resolution);
          grid_1_test.x=int(x_test/resolution);
          grid_2_test.y=i;
          grid_1_test.y=i-1;
          if(check_grid_for_obs(grid_1_test)) {
            return(true);
          }
          if(check_grid_for_obs(grid_2_test)) {
            return(true);
          }
        }
      }
    } else if (difference_y<0) {
      if (angle > -PI/4){
        for(int i = grid_1.x+1; i < grid_1.x+difference_x+1; i++) {

          y_test=(grad*i*resolution)+c;
          grid_2_test.y=int(y_test/resolution);
          grid_1_test.y=int(y_test/resolution);
          grid_2_test.x=i;
          grid_1_test.x=i-1;
          if(check_grid_for_obs(grid_1_test)) {
            return(true);
          }
          if(check_grid_for_obs(grid_2_test)) {
            return(true);
          }
        }
      } else {
        for(int i = grid_1.y; i > grid_2.y; i--) {
          x_test=((i*resolution)-c)/grad;
          grid_2_test.x=int(x_test/resolution);
          grid_1_test.x=int(x_test/resolution);
          grid_2_test.y=i;
          grid_1_test.y=i-1;
          if(check_grid_for_obs(grid_1_test)) {
            return(true);
          }
          if(check_grid_for_obs(grid_2_test)) {
            return(true);
          }
        }
      }

    }
    return(false);
}

bool check_grid_for_obs (path_planning::grid_cell grid) {
    int map_array_value=grid.y*map_width+grid.x;
    if (map[map_array_value] > OBSTACLE_THRESHOLD || map[map_array_value] == -1) {
      return(true);
    } else {
      return (false);
    }
}
	
};
  


	
int main(int argc, char **argv)
{
  //ros setup
	static const int rate=10000;
	ros::init(argc, argv, "path");
	ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::Subscriber sub = nh.subscribe("map", 1000, chatterCallback);
	ros::Rate r(rate);

	while (map_loaded_flag){
		ros::spinOnce();
		sleep(1);
	}
	const double upper_x=66;
	const double lower_x=10;
	const double upper_y=64;
	const double lower_y=9;
	std::uniform_real_distribution<double> unif_x(lower_x,upper_x);
	std::uniform_real_distribution<double> unif_y(lower_y,upper_y);

  
  //initialize RRT object and variables
	static const double child_distance=0.5;
	static const double x_start=52;
	static const double y_start=50; 
	static const double x_goal=29;
	static const double y_goal=20;
	static const double resolution=0.05; 
	static const double radius_goal=0.5;
	static const int radius_neighbour = 1;
	RRT path_planning(child_distance,x_start,y_start,resolution,x_goal,y_goal,radius_goal,radius_neighbour);  //would intialize path planner to have root at robot base
	geometry_msgs::Point next_point,parent,rand_point;
	int index_of_closest=0;
	int index_of_lowest_cost_neighbour=0;
	int map_array_value=0;
	  
  
  //marker setyp		
	visualization_msgs::Marker points, line_list, goal, path_points, path;
	points.header.frame_id=line_list.header.frame_id= goal.header.frame_id = path_points.header.frame_id = path.header.frame_id = "/my_frame";
	points.header.stamp= line_list.header.stamp = goal.header.stamp = path_points.header.stamp = path.header.stamp = ros::Time::now();
	points.ns="nodes";
	goal.ns="goal_node";
	line_list.ns="edges";
	path_points.ns="path_points_nodes";
	path.ns="path_nodes";
	points.action=line_list.action=goal.action = path_points.action = path.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w=line_list.pose.orientation.w=goal.pose.orientation.w= path_points.pose.orientation.w = path.pose.orientation.w = 1.0;

	points.id=0;
	line_list.id=1;
	goal.id=3;
	path_points.id=4;
	path.id=5;

	points.type = visualization_msgs::Marker::POINTS;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	goal.type = visualization_msgs::Marker::POINTS;
	path_points.type = visualization_msgs::Marker::POINTS;
	path.type = visualization_msgs::Marker::LINE_LIST;

	points.scale.x = 0.05;
	points.scale.y = 0.05;
	goal.scale.x = 0.1;
	goal.scale.y = 0.1;
	path_points.scale.x = 0.1;
	path_points.scale.y = 0.1;

	line_list.scale.x = 0.01;
	path.scale.x = 0.1;

	points.color.b = 1.0f;
	points.color.a = 1.0;
	goal.color.g = 1.0f;
	goal.color.a = 1.0;
	path_points.color.a= 1.0;
	path_points.color.r = 1.0f;
	
	path.color.a= 1.0;
	path.color.r = 1.0f;
	line_list.color.g = 1.0;
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



		//ROS_INFO("New Point");
		
		if (!path_planning.is_goal_found()){
			rand_point=path_planning.get_rand_point(unif_x,unif_y);
			index_of_closest=path_planning.find_closest(rand_point); // returns all_nodes postion of closest
			next_point=path_planning.new_point(path_planning.get_all_nodes_value(index_of_closest),rand_point); // find point that could be added to tree
			map_array_value=path_planning.convert_grid_cell(path_planning.find_grid_cell(next_point)); // get point of last value in node pointer array to check if in obstacle
			//ROS_INFO("Array value is %d",array_grid);

			//if statement to implement obstacle avoidance, could make function in RRT class to do this;
			if (map[map_array_value] < OBSTACLE_THRESHOLD || map[map_array_value] == -1) {
				index_of_lowest_cost_neighbour=path_planning.check_neighbours(next_point,index_of_closest);
				if(index_of_lowest_cost_neighbour !=-1){	//only enter if suitable neighbour found							
					path_planning.add_node_to_tree(index_of_lowest_cost_neighbour,next_point);
					points.points.push_back(next_point);
					line_list.points.push_back(path_planning.get_all_nodes_value(index_of_lowest_cost_neighbour));
					line_list.points.push_back(next_point);
					marker_pub.publish(points);
					marker_pub.publish(goal);
					marker_pub.publish(line_list);
					//ROS_INFO("Point placement complete");
					//ROS_INFO(" ");
				}
			} 
		} else {
			path_planning.find_path();
			path_planning.print_path();
			ROS_INFO("found and printed path");
			for (int i = path_planning.path_length()-2; i >=0 ;i--){
					path.points.push_back(path_planning.get_path_point(i));
					path.points.push_back(path_planning.get_path_point(i+1));	
					marker_pub.publish(path);
			}			
			ROS_INFO("path cost = %lf ",path_planning.get_cost(path_planning.get_I()-1) );
			return(1);
		}

		//block loop
		//while(1)

		r.sleep();
	}   
}














































