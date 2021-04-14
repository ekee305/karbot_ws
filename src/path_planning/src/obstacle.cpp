
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <array>
#include <geometry_msgs/Point.h>
#include <random>

std::default_random_engine re;



void chatterCallback(const nav_msgs::OccupancyGrid &msg)
{
  ROS_INFO("number of elements %d", int(msg.data.size()));
  std::vector<int> map;
  for(int i = 0; i <= msg.data.size(); i++) {
    map.push_back(msg.data[i]);
    ROS_INFO("map %d = [%d]", i, map[i]);
  }
  ROS_INFO("end of callback");
}

geometry_msgs::Point find_grid_cell(geometry_msgs::Point new_point);
double get_rand_point(std::uniform_real_distribution<double> unif);





double resolution=1;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle");
	ros::NodeHandle n;
  std::uniform_real_distribution<double> unif(1,10);
  /*geometry_msgs::Point test;
  test.x=9.5;
  test.y=9.5; 

  geometry_msgs::Point grid_cell;*/



  ros::Subscriber sub = n.subscribe("map", 1000, chatterCallback);
  while (1) {
    ros::spinOnce();
    ROS_INFO("random Number = %lf",get_rand_point(unif));
  }
  //ros::spin();
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
