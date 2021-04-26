
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <array>
#include <geometry_msgs/Point.h>
#include <random>
#include "map_msgs/OccupancyGridUpdate.h" 

std::default_random_engine re;
double resolution_from_map;
int map_width, map_height;
std::vector<int> map;
bool map_loaded_flag= false;



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


void mapUpdateCallback(const map_msgs::OccupancyGridUpdate &msg){
	int k=0;
	for(int j = 0; j < msg.height; j++){
		for(int i=0; i < msg.width; i++){
			map[(msg.y+j)*map_width+(msg.x+i)]=msg.data[k++];
		}
	}
	ROS_INFO("Map updated");
}

geometry_msgs::Point find_grid_cell(geometry_msgs::Point new_point);
double get_rand_point(std::uniform_real_distribution<double> unif);

//std::vector<geometry_msgs::Point> map[10][10];




double resolution=1;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle");
	ros::NodeHandle nh;
  ros::NodeHandle n;
  ros::Subscriber sub = nh.subscribe("/global_costmap_node/costmap/costmap", 1000, chatterCallback);
  ros::Subscriber map_update_sub = n.subscribe("/global_costmap_node/costmap/costmap_updates", 1000, mapUpdateCallback);

  /*std::uniform_real_distribution<double> unif(1,10);
  geometry_msgs::Point test1;
  test1.x=0.5;
  test1.y=0.5; 

  geometry_msgs::Point test2;
  test2.x=9.5;
  test2.y=9.5; 

  map[0][0].push_back(test1);
  map[0][0].push_back(test2);


  ros::spin();
  for (int i =0; i < map[0][0].size();i++){
    ROS_INFO("test point in grid (0,0) is (%lf,%lf)",map[0][0].at(i).x,map[0][0].at(i).y);
  }*/
  while(1){
    ros::spinOnce();
  }
}

geometry_msgs::Point find_grid_cell(geometry_msgs::Point new_point) {
    geometry_msgs::Point grid;
    grid.x= new_point.x/resolution;
    grid.y= new_point.y/resolution;
    ROS_INFO("grid is (%d,%d)",int(grid.x),int(grid.y));
    return(grid);
}

int convert_grid_cell (geometry_msgs::Point grid) {
  int array=(grid.y*10)+grid.x;
  return(array);

}


double get_rand_point(std::uniform_real_distribution<double> unif){
  double a_random_double = unif(re);
  return (a_random_double);
}
