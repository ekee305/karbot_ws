		// add grid based spacial indexing
//replace with reolution that is given with map data.
//fix hard coded grid width
//fix rand point cause it ain't rand
//make distance function
//remeber to change grid width stuff as well and area of grid
// deal with goal outside map
//server to request map


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
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>
#include <chrono>
#include <queue>
#include "map_msgs/OccupancyGridUpdate.h" 




#define PI 3.14159265
#define OBSTACLE_THRESHOLD 20.0
#define GRID_WIDTH 10
#define GRID_HEIGHT 10
#define GRID_RESOLUTION 1
#define SEARCH_AREA 100

//test for branch
//lol

//Global variables
std::vector<int> map;
double resolution_from_map;

int map_width,map_height;
geometry_msgs::Point position;
double roll,pitch, yaw;

std::random_device rd;
std::default_random_engine re(rd());
geometry_msgs::Point goal;
bool goal_received=false;
bool debugging=true;
bool first_pose_loaded=false;
bool map_loaded_flag=false;
bool display=false;


void chatterCallback(const nav_msgs::OccupancyGrid &msg) 
{
	ROS_INFO("number of elements %d", int(msg.data.size()));
	map.insert(map.end(), &msg.data[0], &msg.data[msg.data.size()]);
	ROS_INFO("Map loaded");
	resolution_from_map=msg.info.resolution;
	map_width=msg.info.width;
	map_height=msg.info.height;
	map_loaded_flag=true;
}

void goalCallback(const geometry_msgs::PoseStamped &msg) 
{
	if(goal.x != msg.pose.position.x && goal.y != msg.pose.position.y){
		goal.x=msg.pose.position.x;
    	goal.y=msg.pose.position.y;
		goal_received=true;
	} 
}

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) //might be quicker way of loading map by simply equating to data
{
	position.x=msg->pose.pose.position.x;
    position.y=msg->pose.pose.position.y;
    tf::Quaternion q( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    first_pose_loaded=true;

}

void mapUpdateCallback(const map_msgs::OccupancyGridUpdate &msg){
	if(map_loaded_flag){
		int k=0;
		for(int j = 0; j < msg.height; j++){
			for(int i=0; i < msg.width; i++){
				map[(msg.y+j)*map_width+(msg.x+i)]=msg.data[k++];
			}
		}
		//ROS_INFO("Map updated");
	}
}


struct node 
{
	int ID;
	geometry_msgs::Point point; 
	double cost;
	node* parent;
	std::vector<node*> children;
};

class RRT {
public:
	geometry_msgs::Point dummy_point;
	
private:
	double distance,goal_radius,path_cost,neighbour_radius,node_dist;
	double map_resolution; // get this from the map data coming from the server.
	double grid_resolution;
	int density;


	std::vector<geometry_msgs::Point> path;


	geometry_msgs::Point root_point;	

	node* root;
	node* goal_node;
	std::vector<node*> neighbours;
	std::vector<node*> node_list;
	std::vector<node*> goal_list;
	std::vector<node*> spatial_grid[31][31];
	std::vector<node*> path_nodes;
	std::deque<node*> qr;
	std::deque<node*> qs;
	
	int I;
	bool goal_found;


public:
	RRT(double d, double x_start, double y_start, double r,double g_r, double goal_r,int den, double neighbour_r, double rs) : distance (d), map_resolution(r),goal_radius(goal_r), density(den), neighbour_radius(neighbour_r),grid_resolution(g_r),node_dist(rs) {
		root_point.x=x_start;
		root_point.y=y_start;		
		srand((unsigned int)time(NULL));
		I=0;
		create_root();
		goal_found=false;
		path_cost=INFINITY;
		dummy_point.x=-1000;
		dummy_point.y=-1000;
	}

	void create_root() 
	{
		node* new_node= new node;
		new_node->ID = I++;
		new_node->parent = NULL;
		new_node->cost = 0;
		new_node->point.x=root_point.x;
		new_node->point.y=root_point.y;
		node_list.push_back(new_node);
		add_to_spatial_index(new_node);
		root = new_node;
	}

	node* add_node_to_tree(node* parent, geometry_msgs::Point new_point) 
	{
		if (debugging){
			ROS_INFO("I've entered add_node_to_tree");
		}
		node* new_node = new node;			
		new_node->ID = I++;
		new_node->point.x=new_point.x;
		new_node->point.y=new_point.y;
		new_node->parent = parent;
		new_node->cost = calculate_cost(parent,new_node);
		update_children(parent, new_node);
		node_list.push_back(new_node);
		add_to_spatial_index(new_node);
		return(new_node);
	}

	double calculate_cost (node* parent_node, node* child_node) //add check if node is in obstacle
	{
		double cost;
		if (check_line_obstacle(parent_node->point,child_node->point)) {
			cost=INFINITY;	
		} else if(parent_node->parent == NULL) {
			cost=get_dist(parent_node->point,child_node->point);
		} else {
			cost=calculate_cost(parent_node->parent,parent_node)+get_dist(parent_node->point,child_node->point); 
		}
		return(cost);
	}

	double get_cost(node *temp_node)
	{
		double cost;
		if(temp_node->parent==NULL){
			cost=root->cost;
		} else {
			cost=calculate_cost(temp_node->parent,temp_node);
			temp_node->cost=cost;
		}
		return(cost);
	}

	void update_children(node* parent, node *child) 
	{
		parent->children.push_back(child);
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

	geometry_msgs::Point get_root(){
		return(root_point);
	}
	
	geometry_msgs::Point get_goal() {
		return(goal);
	}
	
	bool get_goal_found(){
		return(goal_found);
	}
	
	node* get_goal_node(){
		return(goal_node);
	}
	
	double get_goal_node_cost(){
		return(get_cost(goal_node));
	}
	
	std::vector<node*> get_node_list(){
		return(node_list);
	}

	node* get_node_list_element(int index){
		return(node_list[index]);
	}
	
	geometry_msgs::Point get_rand_point(std::uniform_real_distribution<double> unif_x, std::uniform_real_distribution<double> unif_y){	
		geometry_msgs::Point point;
		point.x=unif_x(re);
		point.y=unif_y(re);
		//ROS_INFO(" rand point = (%lf, %lf)",point.x, point.y);
		return (point);
	}
	
 	geometry_msgs::Point new_point(node* closest, geometry_msgs::Point random_point){
		if (debugging){
			ROS_INFO("I've entered new_point");
		}
		double angle,x,y;
		x=random_point.x-closest->point.x;
		y=random_point.y-closest->point.y;
		geometry_msgs::Point next_point;
		angle=atan2(y,x);
		next_point.x=closest->point.x+distance*cos(angle);
		next_point.y=closest->point.y+distance*sin(angle);
		//ROS_INFO("parent is (%f,%f), child is (%f,%f)",closest->point.x,closest->point.y,next_point.x, next_point.y);
		return(next_point);
	}

	node* check_neighbours (geometry_msgs::Point new_point){
		if (debugging){
			ROS_INFO("I've entered check neighbours");
		}
		neighbours.clear();
		double cost_from_neighbour,total_cost;
		double lowest_cost=INFINITY;
		double x_diff,y_diff;
		find_neighbours(new_point);
		update_neighbour_radius();
		node* closest_neighbour=NULL;
		for (int i = 0;i<neighbours.size();i++){
			x_diff=new_point.x-neighbours[i]->point.x;
			y_diff=new_point.y-neighbours[i]->point.y;
			if (abs(x_diff) <= neighbour_radius && abs(y_diff)<=neighbour_radius){
				cost_from_neighbour=sqrt(pow(y_diff,2)+pow(x_diff,2));
				if(cost_from_neighbour <= neighbour_radius && !check_line_obstacle(new_point,neighbours[i]->point)){
					total_cost=get_cost(neighbours[i])+cost_from_neighbour;
					if (total_cost < lowest_cost){
						closest_neighbour=neighbours[i];
						lowest_cost=total_cost;
					}
				}
			}	 
		}

		return(closest_neighbour);
	}
	
	void rewire_root_neighbours (node* test_node) {
		if (debugging){
			ROS_INFO("I've entered rewire root neighbours");
		}
		
		double total_cost;
		if (check_grid_for_obs(find_grid_cell(test_node->point))){
			test_node->cost=INFINITY;
			ROS_INFO("I've exited rewire root neighbours");
			return;
		}
		update_neighbour_radius();
		find_neighbours(test_node->point);
		for (int i = 0;i<neighbours.size();i++){
			if(get_dist(neighbours[i]->point,test_node->point) < neighbour_radius){	
				total_cost=calculate_cost(test_node,neighbours[i]);
				if (total_cost < get_cost(neighbours[i]) && !check_line_obstacle(test_node->point,neighbours[i]->point)){
					remove_child_from_parent(neighbours[i]);
					neighbours[i]->parent=test_node;
					neighbours[i]->cost=total_cost;
					if(!check_qs(neighbours[i])){
						add_to_root_queue(neighbours[i]);
					}
				}
			}
		}
		ROS_INFO("i am here 1");
		if(test_node!=root){
			ROS_INFO("i am here 2");
			if (check_line_obstacle(test_node->parent->point,test_node->point)){
				ROS_INFO("i am here 3");
				test_node->cost=INFINITY;
			}
		}
		if (debugging){
			ROS_INFO("I'm leaving rewire neighbours");
		}
		return;
	}

	void rewire_random_node_neighbours (node* test_node) {
		if (debugging){
			ROS_INFO("I've entered rewire random node neighbours");
		}
		if (check_grid_for_obs(find_grid_cell(test_node->point))){
			test_node->cost=INFINITY;
			return;
		}
		double total_cost;
		find_neighbours(test_node->point);
		update_neighbour_radius();
		for (int i = 0;i<neighbours.size();i++){
			if(get_dist(neighbours[i]->point,test_node->point) < neighbour_radius){	
				total_cost=calculate_cost(test_node,neighbours[i]);
				if (total_cost < get_cost(neighbours[i]) && !check_line_obstacle(test_node->point,neighbours[i]->point)){
					remove_child_from_parent(neighbours[i]);
					neighbours[i]->parent=test_node;
					neighbours[i]->cost=total_cost;
					add_to_random_queue(neighbours[i]);	
				}
			}
		}
		if(test_node!=root){
			if (check_line_obstacle(test_node->parent->point,test_node->point)){
				test_node->cost=INFINITY;
			}
		}
		if (debugging){
			ROS_INFO("I'm leaving rewire neighbours");
		}
		return;
	} 

	node* find_closest(geometry_msgs::Point random_point){
		if (debugging){
			ROS_INFO("I've entered find_closest");
		}
		int near=0;
		double temp=0;
		double mag=INFINITY;
		double x_diff,y_diff;
		node* nearest_node;
		//ROS_INFO("1random point is (%lf,%lf)",random_point.x,random_point.y);
		find_neighbours(random_point);


		for(int i = 0;i<neighbours.size();i++){
			//ROS_INFO("checked neighbours (%.2lf,%.2lf)",neighbours[i]->point.x,neighbours[i]->point.y);
			x_diff=random_point.x-neighbours[i]->point.x;
			y_diff=random_point.y-neighbours[i]->point.y;
			temp=sqrt(pow(y_diff,2)+pow(x_diff,2));

			if(temp<mag){
				mag=temp;
				nearest_node=neighbours[i];
			}    
		}
		//ROS_INFO("closest node is (%.2lf,%.2lf)",nearest_node->point.x,nearest_node->point.y);

		return (nearest_node);  
	}

	void find_neighbours(geometry_msgs::Point temp_point){ //add exception handling about if no point found
		if (debugging){
			ROS_INFO("I've entered find_neighbours");
		}
		int lower_x_grid,upper_x_grid,lower_y_grid,upper_y_grid;
		path_planning::grid_cell grid=find_grid_cell_for_spatial_indexing(temp_point);
		//ROS_INFO("test point is (%lf,%lf)",temp_point.x,temp_point.y);
		//ROS_INFO("grid is (%d,%d)",grid.x,grid.y);
		neighbours.clear();

		/*for (int i =0; i< node_list.size();i++){
			ROS_INFO("nodelist %d (%.2lf,%.2lf)",i,node_list[i]->point.x,node_list[i]->point.y); 
		}*/
		int search_width=0;
		while(neighbours.empty()){
			search_width++;
			if((grid.x-search_width)<20){
				lower_x_grid=20;
			} else {
				lower_x_grid=grid.x-search_width;
			}
			if((grid.x+search_width)>29){
				upper_x_grid=29;
			} else{
				upper_x_grid=grid.x+search_width;
			}
			if((grid.y-search_width)<20){
				lower_y_grid=20;
			} else {
				lower_y_grid=grid.y-search_width;
			}
			if((grid.y+search_width)>29){
				upper_y_grid=29;
			} else {
				upper_y_grid=grid.y+search_width;
			}
			//ROS_INFO("X range (%d,%d)",lower_x_grid,upper_x_grid);
			//ROS_INFO("Y range (%d,%d)",lower_y_grid,upper_y_grid);
			
			for(int i=lower_x_grid;i <= upper_x_grid;i++){
				for (int j=lower_y_grid;j <=upper_y_grid;j++){
					//ROS_INFO("Checking grid (%d,%d)",i,j);
					for(int k=0; k < spatial_grid[i][j].size();k++){
						//ROS_INFO("spacial_index (%d,%d) point (%lf,%lf)",i,j,spatial_grid[i][j].at(k)->point.x,spatial_grid[i][j].at(k)->point.y);
						neighbours.push_back(spatial_grid[i][j].at(k));
					}
				}
			}
		}
		/*ROS_INFO("number of neighbours found %d",neighbours.size());		
		for(int i = 0;i<neighbours.size();i++){
			ROS_INFO("neighbours (%.2lf,%.2lf)",neighbours[i]->point.x,neighbours[i]->point.y);   
		}*/
		if (debugging){
			ROS_INFO("im leaving find neighbour");
		}
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
		int map_array_value=grid.y*map_width+grid.x;
		return(map_array_value);
	}

	int round_to_grid_cell (double coord){
		return(int(coord/map_resolution));
	}

	void is_goal_found(){
		if (debugging){
			ROS_INFO("I've entered is goal found");
		}	
		if (get_dist(root->point,goal) > goal_radius){
			goal_list.clear();
			double x_diff,y_diff,cost_to_goal;
			find_neighbours(goal);
			for (int i = 0;i<neighbours.size();i++){
				x_diff=goal.x-neighbours[i]->point.x;
				y_diff=goal.y-neighbours[i]->point.y;
				if (abs(x_diff) <= goal_radius && abs(y_diff) <= goal_radius){
					cost_to_goal=sqrt(pow(y_diff,2)+pow(x_diff,2));
					if(cost_to_goal <= goal_radius && !check_line_obstacle(goal,neighbours[i]->point)){
						goal_found=true;
						goal_node=neighbours[i];
						goal_list.push_back(neighbours[i]);
					}
				}	 
			}
		} else {
			goal_found=false;
		}
		if (debugging){
			ROS_INFO("I've exited is goal found");
		}	
		return;
	}

	std::vector<geometry_msgs::Point> find_path() {
		if (debugging){
			ROS_INFO("I've entered find path");
		}	
		//path_cost=get_cost(goal_node)+get_dist(goal_node->point,goal);	
		for (int i=0;i<goal_list.size();i++){
			if ((get_cost(goal_node)+get_dist(goal_node->point,goal)) > (get_cost(goal_list[i])+get_dist(goal_node->point,goal))){
				goal_node=goal_list[i];
			}
		}
		
		if (path_cost > (goal_node->cost + get_dist(goal_node->point,goal))){
			path_cost=goal_node->cost + get_dist(goal_node->point,goal);
			geometry_msgs::Point temp_point;
			path.clear();
			path_nodes.clear();
			node* node_ptr = goal_node;
			temp_point.x=node_ptr->point.x;
			temp_point.y=node_ptr->point.y;
			path.push_back(temp_point);
			path_nodes.push_back(node_ptr);
			while(node_ptr->parent != NULL) 
			{
				node_ptr=get_parent(node_ptr);
				temp_point.x=node_ptr->point.x;
				temp_point.y=node_ptr->point.y;
				path.push_back(temp_point);
				path_nodes.push_back(node_ptr);
			}
		} 
		if (debugging){
			ROS_INFO("I've exited find path");
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
	/*if (debugging){
		ROS_INFO("I've entered check line obstacle");
	}*/
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
	/*if (debugging){
		ROS_INFO("I've exited check line obstacle");
	}*/
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
		if (debugging){
			ROS_INFO("I've entered add_to_spatial_index");
		}
		path_planning::grid_cell grid=find_grid_cell_for_spatial_indexing(temp_node->point);
		spatial_grid[grid.x][grid.y].push_back(temp_node);
	}

	void update_neighbour_radius(){
		neighbour_radius=sqrt((SEARCH_AREA*double (density))/(PI*double(node_list.size())));
		if (neighbour_radius<node_dist){
			neighbour_radius=node_dist;
		}
	}

	bool check_node_density(){
		if (neighbours.size() > density){
			return (false);
		} else {
			return(true);
		}
	}

	bool node_dist_check(geometry_msgs::Point test_point, node* closest) {
		double dist=get_dist(closest->point,test_point);
		if (dist > node_dist){
			return(true);
		} else {
			return(false);
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

	void change_root(node* new_root){
		int node_list_index;
		remove_child_from_parent(new_root);
		new_root->children.push_back(root);
		root->parent=new_root;
		new_root->parent=NULL;
		new_root->cost=0;
		root->cost=calculate_cost(root->parent,root);
		for(int i=0;i<node_list.size();i++){
			if(node_list[i]==new_root){
				node_list_index=i;
			}
		}
		node_list[0]=new_root;
		node_list[node_list_index]=root;
		root=new_root;
		root_point.x=root->point.x;
		root_point.y=root->point.y;
		//rewire_neighbours(root);
	}

	void print_next_path_point(){
		if 	(path.size()==1){
			ROS_INFO("travelling to point (%lf,%lf)",path[0].x,path[0].y);
		} else {
			ROS_INFO("travelling to point (%lf,%lf)",path[path.size()-2].x,path[path.size()-2].y);
		}
	}

	geometry_msgs::Point get_next_path_point(){
		geometry_msgs::Point temp_point;
		if(path.size()!=0){
			temp_point=path[path.size()-2];
		} else {
			temp_point=dummy_point;
		}
		return(temp_point);
	}

	node* get_next_path_node(){
		node* temp_node;
		if (path_nodes.size()==1){
			temp_node=path_nodes[0];
		} else {
			temp_node=path_nodes[path_nodes.size()-2];
		}
		return(temp_node);
	}

	node* get_root_node(){
		return(root);
	}

	std::vector<node*> get_path_nodes(){
		return(path_nodes);
	}

	void clear_goal_variables(){
		goal_node=NULL;
		goal_list.clear();
		goal_found=false;
	}

	void clear_path_variables() {
		path.clear();
		path_nodes.clear();
		path_cost=INFINITY;
	}

	void rewire_random_nodes(){
		for(int i=0;i<5;i++){
		if(qr.empty()){
			break;
		}
		rewire_random_node_neighbours(qr.front());
		qr.pop_front();
		}
	}
	
	void rewire_from_root(){
		for(int i = 0;i<10;i++){
			if(qs.empty()){
				qs.push_back(root);
			}
			rewire_root_neighbours(qs.front());
			qs.pop_front();
		}
	}

	void add_to_random_queue(node* temp_node){
		qr.push_back(temp_node);
	}

	void add_to_root_queue(node* temp_node){
		qs.push_back(temp_node);
	}

	void clear_qs(){
		qs.clear();
	}

	bool check_qs(node* temp_node){
		bool is_in_qs=false;
		for(int i=0;i<qs.size();i++){
			if(temp_node==qs[i]){
				is_in_qs=true;
				break;
			}
		}
		return(is_in_qs);
	}

	void rewire_neighbours (node* test_node) {
		if (debugging){
			ROS_INFO("I've entered rewire neighbours");
		}
		double total_cost;
		for (int i = 0;i<neighbours.size();i++){
			total_cost=calculate_cost(test_node,neighbours[i]);
			if (total_cost < get_cost(neighbours[i]) && !check_line_obstacle(test_node->point,neighbours[i]->point)){
				remove_child_from_parent(neighbours[i]);
				neighbours[i]->parent=test_node;
				neighbours[i]->cost=total_cost;	
			}
		}
		//ROS_INFO(" ");
		return;
	} 

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
	static const int rate=1000;
	ros::init(argc, argv, "path");
	ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::NodeHandle p;
	ros::NodeHandle g;
	ros::NodeHandle a;
	ros::NodeHandle m;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::Publisher path_pub = p.advertise<geometry_msgs::Point>("/next_point_on_path", 10);
	ros::Subscriber sub = nh.subscribe("/global_costmap_node/costmap/costmap", 1, chatterCallback);
	//ros::Subscriber sub = nh.subscribe("map", 1000, chatterCallback);
	ros::Subscriber amcl_sub = a.subscribe("/amcl_pose", 1, amclCallback);
	ros::Subscriber goal_sub = g.subscribe("/goal", 1, goalCallback);
	ros::Subscriber map_update_sub = m.subscribe("/global_costmap_node/costmap/costmap_updates", 1, mapUpdateCallback);
	ros::Rate r(rate);

	path_planning::path_to_goal path_to_publish;

	//waiting ons ubscribers
	while (!map_loaded_flag){
		ROS_WARN_ONCE("Waiting for map");
		ros::spinOnce();
		sleep(1);
	}
	ROS_WARN_ONCE("Map recieved");
    while(!first_pose_loaded){
        ROS_WARN_ONCE("Waiting for AMCL pose");
		ros::spinOnce();
    }
	ROS_WARN_ONCE("Pose recieved");

	/*while (!goal_received){
		ROS_WARN_ONCE("Please select goal point");
		ros::spinOnce();
	}
	ROS_INFO("goal recieved: (%lf,%lf)",goal.x,goal.y);*/
	goal.x=position.x;
	goal.y=position.y;

	// change to be flexible with map size
	const double upper_x=30;  
	const double lower_x=20; 
	const double upper_y=30;
	const double lower_y=20;   
	std::uniform_real_distribution<double> unif_x(lower_x,upper_x);
	std::uniform_real_distribution<double> unif_y(lower_y,upper_y);

  
  //initialize RRT object and variables
	static const double child_distance=0.75;
	static const int density_of_nodes=50;
	static const double x_start=position.x; 
	static const double y_start=position.y;  
	//static const double x_start=25; 
	//static const double y_start=25;  

	static const double map_resolution=0.05;
	static const double grid_resolution=GRID_RESOLUTION;
	static const double radius_goal=0.3;
	static const int radius_neighbour = 0.75;
	static const double dist_node = 0.75;
	RRT path_planning(child_distance,x_start,y_start,map_resolution,grid_resolution,radius_goal,density_of_nodes,radius_neighbour,dist_node);  //would intialize path planner to have root at robot base
	geometry_msgs::Point next_point,parent,rand_point;
	node* closest_node;
	node* lowest_cost_neighbour;
	node* new_node;
	node* new_root;
	int map_array_value=0;
	std::vector<geometry_msgs::Point> temp_path,path_to_goal;

	//intialise robot pose class
	robot_pose robot_postion;
	  
  
  //marker setyp		
	visualization_msgs::Marker points, line_list, goal_marker, path_points, path;
	points.header.frame_id=line_list.header.frame_id= goal_marker.header.frame_id = path_points.header.frame_id = path.header.frame_id = "map";
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
	goal_marker.scale.x = 0.15;
	goal_marker.scale.y = 0.15;
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


 
	points.points.push_back(path_planning.get_root());
	goal_marker.points.push_back(goal);
	marker_pub.publish(goal_marker);
	

	//timer setup
	std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
	std::chrono::duration<double, std::milli> Elapsed;

	time_t timer;
	int begin = time(&timer);

    
	while(ros::ok()){
		ROS_INFO("restarted algorithm");
		ROS_INFO(" ");
		
		ros::spinOnce();
		//find rand point
		if(goal_received){
			path_planning.clear_goal_variables();
			path.points.clear();
			path_planning.clear_path_variables();
			marker_pub.publish(path);
			goal_received=false;
		}
		start_time = std::chrono::system_clock::now();
		end_time = std::chrono::system_clock::now();
		Elapsed = end_time - start_time;
		while (Elapsed.count() < 20.0){
			rand_point=path_planning.get_rand_point(unif_x,unif_y);
			closest_node=path_planning.find_closest(rand_point); // returns pointer to closest node
			next_point=path_planning.new_point(closest_node,rand_point); // find point that could be added to tree
			map_array_value=path_planning.convert_grid_cell(path_planning.find_grid_cell(next_point)); // get point of last value in node pointer array to check if in obstacle
			//ROS_INFO("Array value is %d",array_grid);

			//if statement to implement obstacle avoidance, make function in RRT class to do this;
			if ((map[map_array_value] < OBSTACLE_THRESHOLD || map[map_array_value] == -1)) {

				lowest_cost_neighbour=path_planning.check_neighbours(next_point); //find nearest suitable neighbour
				if (lowest_cost_neighbour !=NULL && (path_planning.check_node_density() || path_planning.node_dist_check(next_point,lowest_cost_neighbour))){	//only enter if suitable neighbour found							
					new_node=path_planning.add_node_to_tree(lowest_cost_neighbour,next_point); // add new node to tree with neighbour of least cost as parent
					path_planning.add_to_random_queue(new_node);
					//path_planning.rewire_neighbours(new_node);
		
				} else if (lowest_cost_neighbour != NULL ){
					path_planning.add_to_random_queue(lowest_cost_neighbour);
				}
			}
			
			path_planning.rewire_random_nodes(); 
			path_planning.rewire_from_root();

			end_time = std::chrono::system_clock::now();
			Elapsed = end_time - start_time;
		}

				//marker updates
					
			
		if (display){
			if (debugging){
				ROS_INFO("I've entered markers");
			}	
			points.action = visualization_msgs::Marker::DELETEALL;
			line_list.points.clear();
			points.points.clear();
			goal_marker.points.clear();
			points.action = visualization_msgs::Marker::ADD;
			points.points.push_back(path_planning.get_node_list_element(0)->point);					
			points.points.push_back(path_planning.get_node_list_element(0)->point);					
			points.points.push_back(path_planning.get_node_list_element(0)->point);					
			for(int i=1;i<path_planning.get_node_list().size()-1;i++){
				points.points.push_back(path_planning.get_node_list_element(i)->point);
				if(path_planning.get_node_list_element(i)->cost!=INFINITY){
					line_list.points.push_back(path_planning.get_node_list_element(i)->point);
					line_list.points.push_back(path_planning.get_node_list_element(i)->parent->point);
				}
			}
			goal_marker.points.push_back(goal);
			marker_pub.publish(points);
			marker_pub.publish(line_list);
			marker_pub.publish(goal_marker);

			if (debugging){
				ROS_INFO("I've exited markers");
			}
		}

		path_planning.is_goal_found();
		if (path_planning.get_goal_found()){
				temp_path.clear();
				temp_path=path_planning.find_path();
				if(temp_path != path_to_goal && temp_path.size() > 1){
					path_to_goal=temp_path;
					path.points.clear();
					for (int i = path_planning.path_length()-2; i > 0 ;i--){
							path.points.push_back(path_planning.get_path_point(i));
							path.points.push_back(path_planning.get_path_point(i+1));	
							marker_pub.publish(path);
					}
					if(path_planning.get_goal_node_cost()!=INFINITY){
						path_pub.publish(path_planning.get_next_path_point());
					} else {
						ROS_INFO("published dummy point 1");
						path_pub.publish(path_planning.dummy_point);
						path_planning.rewire_from_root();
					}
				} else if (temp_path.size()==1) {
					path.points.clear();
					marker_pub.publish(path);
				}
				if (path_planning.get_dist(position,path_planning.get_next_path_point()) < 0.2){
					ROS_INFO("i get here 1");
					new_root=path_planning.get_next_path_node();
					ROS_INFO("i get here 2");
					//ROS_INFO("new_root is (%lf,%lf)",new_root->point.x,new_root->point.y);
					//ROS_INFO("current_root is (%lf,%lf)",path_planning.get_root_node()->point.x,path_planning.get_root_node()->point.y);
					if (path_planning.get_root_node() != path_planning.get_goal_node()  /*&& path_planning.get_cost(new_root) != INFINITY*/){
						ROS_INFO("i get here 3");
						if (path_planning.get_root_node()!= path_planning.get_next_path_node()) {
							path_planning.change_root(path_planning.get_next_path_node());
							ROS_INFO("i get here 4");
							path_planning.clear_qs();
							ROS_INFO("i get here 5");
							path_planning.rewire_from_root();
						}
					} else {
						ROS_INFO("published dummy point 2");
						path_pub.publish(path_planning.dummy_point);
						path_planning.rewire_from_root();
					}
				}			
		} else {
			//ROS_WARN("finding goal or at goal");
			path.points.clear();
			path_planning.clear_path_variables();
			marker_pub.publish(path);
			path_pub.publish(path_planning.dummy_point);
		}
		ros::spinOnce();
		r.sleep();
	}   
}
















































