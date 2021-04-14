
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <array>
#include <geometry_msgs/Point.h>
#include <path_planning/grid_cell.h>

#define PI 3.14159265


std::vector<int> map;
bool map_loaded_flag=1;

void chatterCallback(const nav_msgs::OccupancyGrid &msg) //might be quicker way of loading map by simply equating to data
{
  ROS_INFO("number of elements %d", int(msg.data.size()));
  map.insert(map.end(), &msg.data[0], &msg.data[msg.data.size()]);
  ROS_INFO("Map loaded");
  map_loaded_flag=0;
}

//Function definitions

path_planning::grid_cell find_grid_cell(geometry_msgs::Point new_point);
int convert_grid_cell (path_planning::grid_cell grid);
bool bresenhams_test_bench (geometry_msgs::Point start, geometry_msgs::Point end,std::vector<int> lines_crosses);
bool check_line_obstacle (geometry_msgs::Point point_1, geometry_msgs::Point point_2);
bool check_grid_for_obs (path_planning::grid_cell grid);


double resolution=0.5;// get this from the map data coming from the server.

int main(int argc, char **argv)
{

  ros::init(argc, argv, "obstacle");
	ros::NodeHandle n;
  geometry_msgs::Point start,end;
  geometry_msgs::Point grid_cell;
  start.x = 0.25;
  start.y = 0.25; 
  end.x = 0.25;
  end.y= 1.25;


  ros::Subscriber sub = n.subscribe("map", 1000, chatterCallback);
  while (1) {
    while (map_loaded_flag){
			ros::spinOnce();
	  		sleep(1);
	  }
    ros::spinOnce();
    check_line_obstacle(start, end);
    while(1){}
  }
}
  //ros::spin();


path_planning::grid_cell find_grid_cell(geometry_msgs::Point new_point) {
    path_planning::grid_cell grid;
    grid.x= int(new_point.x/resolution); //replace with reolution that is given with occupancy grid data.
    grid.y= int(new_point.y/resolution);
    //ROS_INFO("grid is (%d,%d)",grid.x,grid.y);
    return(grid);
}

int convert_grid_cell (path_planning::grid_cell grid) {
  int map_array_value=grid.y*10+grid.x;
  return(map_array_value);

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
          //ROS_INFO("X same");
          //ROS_INFO("line crossed = %d",i);
          grid_2_test.x=grid_2.x;
          grid_1_test.x=grid_1.x;
          //ROS_INFO("x_test=%d",grid_1_test.x);
          grid_2_test.y=i;
          grid_1_test.y=i-1;
          //ROS_INFO("tested_grids (%d,%d),(%d,%d)",grid_1_test.x,grid_1_test.y,grid_2_test.x,grid_2_test.y);
          if(check_grid_for_obs(grid_1_test)) {
            //ROS_INFO("obstacle in grid (%d,%d)",grid_1_test.x,grid_1_test.y);
            return(true);
          }
          if(check_grid_for_obs(grid_2_test)) {
            //ROS_INFO("obstacle in grid (%d,%d)",grid_2_test.x,grid_2_test.y);
            return(true);
          }
        }
      } else if (difference_y < 0){
        for(int i = grid_1.y; i > grid_2.y; i--) {
          //ROS_INFO("X same, Y mode entered");
          //ROS_INFO("line crossed = %d",i);
          grid_2_test.x=grid_2.x;
          grid_1_test.x=grid_1.x;
          //ROS_INFO("x_test=%d",grid_1_test.x);
          grid_2_test.y=i;
          grid_1_test.y=i-1;
          //ROS_INFO("tested_grids (%d,%d),(%d,%d)",grid_1_test.x,grid_1_test.y,grid_2_test.x,grid_2_test.y);
          if(check_grid_for_obs(grid_1_test)) {
            //ROS_INFO("obstacle in grid (%d,%d)",grid_1_test.x,grid_1_test.y);
            return(true);
          }
          if(check_grid_for_obs(grid_2_test)) {
            //ROS_INFO("obstacle in grid (%d,%d)",grid_2_test.x,grid_2_test.y);
            return(true);
          }
        }
      }
    } else if (difference_y >= 0) {
      //ROS_INFO("positive gradient");
      if (angle < PI/4){
        for(int i = grid_1.x+1; i < grid_1.x+difference_x+1; i++) {
          //ROS_INFO("line crossed = %d",i);
          y_test=(grad*i*resolution)+c;
          grid_2_test.y=int(y_test/resolution);
          grid_1_test.y=int(y_test/resolution);
          //ROS_INFO("y_test=%d",grid_1_test.y);
          //ROS_INFO("y_diff=%d",difference_y);
          grid_2_test.x=i;
          grid_1_test.x=i-1;
          //ROS_INFO("tested_grids (%d,%d),(%d,%d)",grid_1_test.x,grid_1_test.y,grid_2_test.x,grid_2_test.y);
          if(check_grid_for_obs(grid_1_test)) {
            //ROS_INFO("obstacle in grid (%d,%d)",grid_1_test.x,grid_1_test.y);
            return(true);
          }
          if(check_grid_for_obs(grid_2_test)) {
            //ROS_INFO("obstacle in grid (%d,%d)",grid_2_test.x,grid_2_test.y);
            return(true);
          }
        }
      } else {
        for(int i = grid_1.y+1; i < grid_1.y+difference_y+1; i++) {
          //ROS_INFO("Y mode entered");
          //ROS_INFO("line crossed = %d",i);
          x_test=((i*resolution)-c)/grad;
          grid_2_test.x=int(x_test/resolution);
          grid_1_test.x=int(x_test/resolution);
          //ROS_INFO("x_test=%d",grid_1_test.x);
          grid_2_test.y=i;
          grid_1_test.y=i-1;
          //ROS_INFO("tested_grids (%d,%d),(%d,%d)",grid_1_test.x,grid_1_test.y,grid_2_test.x,grid_2_test.y);
          if(check_grid_for_obs(grid_1_test)) {
            //ROS_INFO("obstacle in grid (%d,%d)",grid_1_test.x,grid_1_test.y);
            return(true);
          }
          if(check_grid_for_obs(grid_2_test)) {
            //ROS_INFO("obstacle in grid (%d,%d)",grid_2_test.x,grid_2_test.y);
            return(true);
          }
        }
      }
    } else if (difference_y<0) {
      //ROS_INFO("negative gradient");
      if (angle > -PI/4){
        for(int i = grid_1.x+1; i < grid_1.x+difference_x+1; i++) {
          //ROS_INFO("line crossed = %d",i);
          y_test=(grad*i*resolution)+c;
          grid_2_test.y=int(y_test/resolution);
          grid_1_test.y=int(y_test/resolution);
          //ROS_INFO("y_test=%d",grid_1_test.y);
          //ROS_INFO("y_diff=%d",difference_y);
          grid_2_test.x=i;
          grid_1_test.x=i-1;
          //ROS_INFO("tested_grids (%d,%d),(%d,%d)",grid_1_test.x,grid_1_test.y,grid_2_test.x,grid_2_test.y);
          if(check_grid_for_obs(grid_1_test)) {
            //ROS_INFO("obstacle in grid (%d,%d)",grid_1_test.x,grid_1_test.y);
            return(true);
          }
          if(check_grid_for_obs(grid_2_test)) {
            //ROS_INFO("obstacle in grid (%d,%d)",grid_2_test.x,grid_2_test.y);
            return(true);
          }
        }
      } else {
        for(int i = grid_1.y; i > grid_2.y; i--) {
          //ROS_INFO("Y mode entered");
          //ROS_INFO("line crossed = %d",i);
          x_test=((i*resolution)-c)/grad;
          grid_2_test.x=int(x_test/resolution);
          grid_1_test.x=int(x_test/resolution);
          //ROS_INFO("x_test=%d",grid_1_test.x);
          grid_2_test.y=i;
          grid_1_test.y=i-1;
          //ROS_INFO("tested_grids (%d,%d),(%d,%d)",grid_1_test.x,grid_1_test.y,grid_2_test.x,grid_2_test.y);
          if(check_grid_for_obs(grid_1_test)) {
            //ROS_INFO("obstacle in grid (%d,%d)",grid_1_test.x,grid_1_test.y);
            return(true);
          }
          if(check_grid_for_obs(grid_2_test)) {
            //ROS_INFO("obstacle in grid (%d,%d)",grid_2_test.x,grid_2_test.y);
            return(true);
          }
        }
      }

    }
    return(false);
}

bool check_grid_for_obs (path_planning::grid_cell grid) {
    int map_array_value=grid.y*10+grid.x;
    if (map[map_array_value] > 30) {
      return(true);
    } else {
      return (false);
    }
}


bool bresenhams_test_bench (geometry_msgs::Point start, geometry_msgs::Point end,std::vector<int> lines_crosses){

//lol

}