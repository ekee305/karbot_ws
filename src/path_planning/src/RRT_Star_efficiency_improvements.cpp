// add grid based spacial indexing
//replace with reolution that is given with map data.
//fix hard coded grid width



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
#include <path_planning/path_to_goal.h>
#include <random>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include <gazebo_msgs/GetModelState.h>
#include <time.h>




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
geometry_msgs::Point goal;
bool goal_recieved=false;


void chatterCallback(const nav_msgs::OccupancyGrid &msg) 
{
	ROS_INFO("number of elements %d", int(msg.data.size()));
	map.insert(map.end(), &msg.data[0], &msg.data[msg.data.size()]);
	ROS_INFO("Map loaded");
	resolution_from_map=msg.info.resolution;
	map_width=msg.info.width;
	map_height=msg.info.height;
	map_loaded_flag=0;
}

void goalCallback(const geometry_msgs::PoseStamped &msg) 
{
	goal.x=msg.pose.position.x;
    goal.y=msg.pose.position.y;
	goal_recieved=true;
}

class RRT {
public:
	
	
private:
	double distance,goal_radius,path_cost,neighbour_radius;
	double map_resolution; // get this from the map data coming from the server.
	double grid_resolution;
	int density;

    std::vector<geometry_msgs::Point> all_nodes;
	std::vector<geometry_msgs::Point> path;


	geometry_msgs::Point start;

	struct node 
	{
		int ID;
		geometry_msgs::Point point; 
		double cost;
		node* parent;
		std::vector<node*> children;
	};	

	node* root;
	node* goal_node;
	std::vector<node*> neighbours;
	std::vector<node*> node_list;
	std::vector<node*> goal_list;
	std::vector<node*> spatial_grid[10][10];
	
	int I;
	bool goal_found;


public:
	RRT(double d, double x_start, double y_start, double r,double g_r, double goal_r,int den, double neighbour_r) : distance (d), map_resolution(r),goal_radius(goal_r), density(den), neighbour_radius(neighbour_r),grid_resolution(g_r) {
		start.x=x_start;
		start.y=y_start;		
		srand((unsigned int)time(NULL));
		I=0;
		create_root();
		goal_found=false;
		path_cost=10000;	
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
		update_children(index, new_node);
		all_nodes.push_back(new_point);
		node_list.push_back(new_node);
	}

	double calculate_cost (node* parent_node, node* child_node) //
	{
		double x_diff;
		double y_diff;
		double cost;
    	x_diff=parent_node->point.x-child_node->point.x;
    	y_diff=parent_node->point.y-child_node->point.y;
    	if(parent_node->parent != NULL){
			cost=calculate_cost(parent_node->parent,parent_node)+sqrt(pow(y_diff,2)+pow(x_diff,2));
		} else {
			cost=sqrt(pow(y_diff,2)+pow(x_diff,2));
		}
		return(cost);
	}

	double get_cost(int index)
	{
		double cost;
		if(node_list[index]->parent!=NULL){		
			cost=calculate_cost(node_list[index]->parent,node_list[index]);
			node_list[index]->cost=cost;
		} else{
			cost=0;
		}
		return(cost);
	}

	double get_cost(node *temp_node)
	{
		double cost;
		if(temp_node->parent!=NULL){
			cost=calculate_cost(temp_node->parent,temp_node);
			temp_node->cost=cost;
		} else {
			cost=0;
		}
		return(cost);
	}

	void update_children(int index, node *child) 
	{
		node_list[index]->children.push_back(child);
	}

	void remove_child_from_parent(node *child){
		node* temp_parent=child->parent;
		for(int i = 0 ;i < temp_parent->children.size(); i++) {
			if (child==temp_parent->children[i]){
				temp_parent->children.erase(temp_parent->children.begin()+i);
				return;
			}
		}
		return;
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

	bool get_goal_found(){
		return(goal_found);
	}
	
	node* get_goal_node(){
		return(goal_node);
	}

	geometry_msgs::Point get_all_nodes_value(int i) {
		return(all_nodes[i]);
	}

	std::vector<node*> get_node_list(){
		return(node_list);
	}

	node* get_node_list_element(int index){
		return(node_list[index]);
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
		neighbours.clear();
		double cost_from_neighbour,total_cost;
		double lowest_cost=1000000;
		double x_diff,y_diff;
		int closest_neighbour_index=-1;
		for(int i = 0;i<all_nodes.size();i++){
			x_diff=new_point.x-all_nodes[i].x;
			y_diff=new_point.y-all_nodes[i].y;
			if (abs(x_diff) <= neighbour_radius && abs(y_diff)<=neighbour_radius){
				cost_from_neighbour=sqrt(pow(y_diff,2)+pow(x_diff,2));
				if(cost_from_neighbour <= neighbour_radius && !check_line_obstacle(new_point,all_nodes[i])){
					neighbours.push_back(node_list[i]);
				}
			}	    
		}
		//find lowest cost path through neighbours
		for (int i = 0;i<neighbours.size();i++){
			x_diff=new_point.x-neighbours[i]->point.x;
			y_diff=new_point.y-neighbours[i]->point.y;
			cost_from_neighbour=sqrt(pow(y_diff,2)+pow(x_diff,2));
			total_cost=get_cost(neighbours[i])+cost_from_neighbour;
			if (total_cost < lowest_cost){
				closest_neighbour_index=neighbours[i]->ID;
				lowest_cost=total_cost;
	
			}
		}
		return(closest_neighbour_index);
	}
	
	void rewire_neighbours () {
		node* test_node=node_list.back();
		double total_cost;
		for (int i = 0;i<neighbours.size();i++){
			total_cost=calculate_cost(test_node,neighbours[i]);
			if (total_cost < get_cost(neighbours[i])){
				remove_child_from_parent(neighbours[i]);
				neighbours[i]->parent=test_node;
				neighbours[i]->cost=total_cost;	
			}
		}
		return;
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
		grid.x= int(new_point.x/map_resolution); 
		grid.y= int(new_point.y/map_resolution);
		//ROS_INFO("grid is (%d,%d)",grid.x,grid.y);
		return(grid);
	}

	path_planning::grid_cell find_grid_cell_for_spatial_indexing(geometry_msgs::Point new_point) {
		path_planning::grid_cell grid;
		grid.x= int(new_point.x/grid_resolution); 
		grid.y= int(new_point.y/grid_resolution);
		//ROS_INFO("grid is (%d,%d)",grid.x,grid.y);
		return(grid);
	}


	int convert_grid_cell (path_planning::grid_cell grid) {
		int map_array_value=grid.y*map_width*map_resolution+grid.x;
		return(map_array_value);
	}

	int convert_grid_cell_for_spatial_indexing (path_planning::grid_cell grid) {
		int grid_array_value=grid.y*10+grid.x;
		return(grid_array_value);
	}

	int round_to_grid_cell (double coord){
		return(int(coord/map_resolution));
	}

	void is_goal_found(){
		double x_diff,y_diff;
		x_diff=goal.x-node_list[I-1]->point.x;
		y_diff=goal.y-node_list[I-1]->point.y;
		double dist_to_goal=sqrt(pow(y_diff,2)+pow(x_diff,2));
		if(dist_to_goal<goal_radius){
			//ROS_INFO("goal found");
		 	goal_found=true;
			goal_node=node_list[I-1];
			goal_list.push_back(node_list[I-1]);
		} 
		return;
	}

	bool at_start(geometry_msgs::Point test) {
		if (test == start){
			return (true);
		} else {
			return (false);
		}
	}

	std::vector<geometry_msgs::Point> find_path() {
		for (int i=0;i<goal_list.size();i++){
			if (get_cost(goal_node) > get_cost(goal_list[i])){
				goal_node=goal_list[i];
			}
		}
		if (path_cost > goal_node->cost){
			path_cost=goal_node->cost;
			geometry_msgs::Point temp_point;
			path.clear();
			path.push_back(goal);
			node* node_ptr= goal_node;
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
		return(path);

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
          y_test=(grad*i*map_resolution)+c;
          grid_2_test.y=int(y_test/map_resolution);
          grid_1_test.y=int(y_test/map_resolution);
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
          x_test=((i*map_resolution)-c)/grad;
          grid_2_test.x=int(x_test/map_resolution);
          grid_1_test.x=int(x_test/map_resolution);
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

          y_test=(grad*i*map_resolution)+c;
          grid_2_test.y=int(y_test/map_resolution);
          grid_1_test.y=int(y_test/map_resolution);
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
          x_test=((i*map_resolution)-c)/grad;
          grid_2_test.x=int(x_test/map_resolution);
          grid_1_test.x=int(x_test/map_resolution);
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
	
	void add_to_spatial_index(node* temp_node){
		path_planning::grid_cell grid=find_grid_cell_for_spatial_indexing(temp_node->point);
		spatial_grid[grid.x][grid.y].push_back(temp_node);
	}

	//void find_adjacent
};

class robot_pose{
public:
	robot_pose(){

	}

	double dist_to_goal(double x_robot,double y_robot, double x_g, double y_g){
			double x_diff,y_diff;
			x_diff=x_g-x_robot;
			y_diff=y_g-y_robot;
			return(sqrt(pow(y_diff,2)+pow(x_diff,2)));
	}

};
  


	
int main(int argc, char **argv)
{


  //ros setup
	static const int rate=100;
	ros::init(argc, argv, "path");
	ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::NodeHandle p;
	ros::NodeHandle g;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::Publisher path_pub = p.advertise<path_planning::path_to_goal>("/path", 10);
	//ros::Subscriber sub = nh.subscribe("/global_costmap_node/costmap/costmap", 1000, chatterCallback);
	ros::Subscriber sub = nh.subscribe("map", 1000, chatterCallback);

	ros::Subscriber goal_sub = g.subscribe("/move_base_simple/goal", 1000, goalCallback);
	ros::Rate r(rate);
	ros::Duration half_sec(0.5);

	path_planning::path_to_goal path_to_publish;

	//map setup
	while (map_loaded_flag){
		ros::spinOnce();
		sleep(1);
	}

	// change to be flexible with map size
	const double upper_x= 10;// 32;  
	const double lower_x=0; //7; 
	const double upper_y=10; //64; 
	const double lower_y=0; //7;   
	std::uniform_real_distribution<double> unif_x(lower_x,upper_x);
	std::uniform_real_distribution<double> unif_y(lower_y,upper_y);


	//set up service to get robot x y points
	/*while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/get_model_state",true);
      ROS_INFO("waiting for get_model_state service");
      half_sec.sleep();
    }

	ros::ServiceClient get_model_state_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	mybot_state.request.model_name = "m2wr";
    mybot_state.request.relative_entity_name = "world";*/

  
  //initialize RRT object and variables
	static const double child_distance=0.5;
	static const int density_of_nodes=5;
	static const double x_start= 5; //19.8; //change to amcl postion;
	static const double y_start= 5;  //52.4; 
	static const double map_resolution=1;
	static const double grid_resolution=1;
	static const double radius_goal=0.5;
	static const int radius_neighbour = 1;
	RRT path_planning(child_distance,x_start,y_start,map_resolution,grid_resolution,radius_goal,density_of_nodes,radius_neighbour);  //would intialize path planner to have root at robot base
	geometry_msgs::Point next_point,parent,rand_point;
	int index_of_closest=0;
	int index_of_lowest_cost_neighbour=0;
	int map_array_value=0;
	std::vector<geometry_msgs::Point> temp_path,path_to_goal;

	//intialise robot pose class
	robot_pose robot_postion;
	  
  
  //marker setyp		
	visualization_msgs::Marker points, line_list, goal_marker, path_points, path;
	points.header.frame_id=line_list.header.frame_id= goal_marker.header.frame_id = path_points.header.frame_id = path.header.frame_id = "my_frame";
	points.header.stamp= line_list.header.stamp = goal_marker.header.stamp = path_points.header.stamp = path.header.stamp = ros::Time::now();
	points.ns="nodes";
	goal_marker.ns="goal_node";
	line_list.ns="edges";
	path_points.ns="path_points_nodes";
	path.ns="path_nodes";
	points.action=line_list.action=goal_marker.action = path_points.action = path.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w=line_list.pose.orientation.w=goal_marker.pose.orientation.w= path_points.pose.orientation.w = path.pose.orientation.w = 1.0;

	points.id=0;
	line_list.id=1;
	goal_marker.id=3;
	path_points.id=4;
	path.id=5;

	points.type = visualization_msgs::Marker::POINTS;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	goal_marker.type = visualization_msgs::Marker::POINTS;
	path_points.type = visualization_msgs::Marker::POINTS;
	path.type = visualization_msgs::Marker::LINE_LIST;

	points.scale.x = 0.05;
	points.scale.y = 0.05;
	goal_marker.scale.x = 0.1;
	goal_marker.scale.y = 0.1;
	path_points.scale.x = 0.1;
	path_points.scale.y = 0.1;

	line_list.scale.x = 0.01;
	path.scale.x = 0.1;

	points.color.b = 1.0f;
	points.color.a = 1.0;
	goal_marker.color.g = 1.0f;
	goal_marker.color.a = 1.0;
	path_points.color.a= 1.0;
	path_points.color.r = 1.0f;
	
	path.color.a= 1.0;
	path.color.r = 1.0f;
	line_list.color.g = 1.0;
	line_list.color.a = 1.0;


 
	while (!goal_recieved){
		ROS_INFO("Please select goal point");
		ros::spinOnce();
	}
	ROS_INFO("goal recieved: (%lf,%lf)",goal.x,goal.y);

	points.points.push_back(path_planning.get_start());
	goal_marker.points.push_back(path_planning.get_goal());
	

	//timer setup
	time_t timer;
	int begin = time(&timer);
    
	while(ros::ok()){
		ros::spinOnce();
		ROS_INFO("I get to 1");
		//find rand point
		rand_point=path_planning.get_rand_point(unif_x,unif_y);
		index_of_closest=path_planning.find_closest(rand_point); // returns all_nodes postion of closest
		next_point=path_planning.new_point(path_planning.get_all_nodes_value(index_of_closest),rand_point); // find point that could be added to tree
		map_array_value=path_planning.convert_grid_cell(path_planning.find_grid_cell(next_point)); // get point of last value in node pointer array to check if in obstacle
		//ROS_INFO("Array value is %d",array_grid);
		ROS_INFO("I get to 2");
		//if statement to implement obstacle avoidance, make function in RRT class to do this;
		if (map[map_array_value] < OBSTACLE_THRESHOLD || map[map_array_value] == -1) {
			ROS_INFO("I get to 3");
			index_of_lowest_cost_neighbour=path_planning.check_neighbours(next_point,index_of_closest); //find nearest suitable neighbour

			if(index_of_lowest_cost_neighbour !=-1){	//only enter if suitable neighbour found							
				path_planning.add_node_to_tree(index_of_lowest_cost_neighbour,next_point); // add new node to tree with neighbour of least cost as parent
				path_planning.rewire_neighbours(); // rewires neighbours of new point;

				//marker updates
				points.action = visualization_msgs::Marker::DELETEALL;
				line_list.points.clear();
				points.points.clear();
				points.action = visualization_msgs::Marker::ADD;
				points.points.push_back(path_planning.get_node_list_element(0)->point);					
				for(int i=1;i<path_planning.get_node_list().size()-1;i++){
					points.points.push_back(path_planning.get_node_list_element(i)->point);
					line_list.points.push_back(path_planning.get_node_list_element(i)->point);
					line_list.points.push_back(path_planning.get_node_list_element(i)->parent->point);
				}
				marker_pub.publish(points);
				marker_pub.publish(line_list);
				marker_pub.publish(goal_marker);
			}
		}

		path_planning.is_goal_found();

		//path to goal found state
		if (path_planning.get_goal_found()){
			temp_path.clear();
			temp_path=path_planning.find_path();
			if(temp_path != path_to_goal){
				path_to_goal=temp_path;
				path.points.clear();
				ROS_INFO("found and printed path");
				path.action = visualization_msgs::Marker::ADD;
				for (int i = path_planning.path_length()-2; i >=0 ;i--){
						path.points.push_back(path_planning.get_path_point(i));
						path.points.push_back(path_planning.get_path_point(i+1));	
						marker_pub.publish(path);
				}			
				ROS_INFO("path cost = %lf ",path_planning.get_goal_node()->cost);//add function to get goal point;
			}		
		}
		//moving to goal state
		if ((time(&timer) - begin) > 45 && path_planning.get_goal_found()){
			path_to_publish.path=path_to_goal;
			path_pub.publish(path_to_publish);
			return(1);

		}
		r.sleep();
	}   
}
















































