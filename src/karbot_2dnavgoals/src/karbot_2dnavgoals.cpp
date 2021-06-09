//Authors: Ethan Kee, Kar Lau
//Date:17/05/2021
//Description: Simple program that can publish goal points on the /goal topic in a loop

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <actionlib/client/simple_action_client.h>
#include <sstream>
#include <geometry_msgs/Point.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>
#include "std_msgs/String.h"
#include "jsoncpp.cpp"
#include <map>
#include <time.h>

using namespace std;

geometry_msgs::Point position;
double roll, pitch, yaw;
bool first_pose_loaded = false;
bool first_message_received = false;

std::deque<geometry_msgs::PoseStamped> high_priority_goals;
std::deque<geometry_msgs::PoseStamped> low_priority_goals;
std::deque<string> high_priority_job_ids;
std::deque<string> low_priority_job_ids;
std::deque<string> completed_job_ids;

map<string, geometry_msgs::PoseStamped> locations;
geometry_msgs::PoseStamped Goal;
std::string current_job_id;
std_msgs::String job_update;

geometry_msgs::PoseStamped ward_a;
geometry_msgs::PoseStamped ward_b;
geometry_msgs::PoseStamped home;
geometry_msgs::PoseStamped lab;

// call back when AMCL postion is recieved
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) //might be quicker way of loading map by simply equating to data
{
  position.x = msg->pose.pose.position.x;
  position.y = msg->pose.pose.position.y;
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  first_pose_loaded = true;
}

void jobRequestCallback(const std_msgs::String::ConstPtr &msg) //might be quicker way of loading map by simply equating to data
{
  Json::Reader reader;
  Json::Reader reader1;
  Json::Value root;

  string json = msg->data.c_str();

  bool parseSuccess = reader.parse(json, root, false);

  if (parseSuccess)
  {
    first_message_received = true;
    const Json::Value numJobRequestsRoot = root["numJobRequests"];
    const Json::Value jobRequestsRoot = root["jobRequests"];
    int numJobReqs = numJobRequestsRoot.asInt();
    cout << "Number of job requests is " << numJobReqs << "\n";
    // cout << "Size of high queue " << high_priority_job_ids.size() << "\n";
    // cout << "Size of low queue " << low_priority_job_ids.size() << "\n";
    for (int index = 0; index <= numJobReqs - 1; index++)
    {

      auto it_high = std::find(high_priority_job_ids.begin(), high_priority_job_ids.end(), jobRequestsRoot[index]["job_id"].asString());
      auto it_low = std::find(low_priority_job_ids.begin(), low_priority_job_ids.end(), jobRequestsRoot[index]["job_id"].asString());
      auto it_completed = std::find(completed_job_ids.begin(), completed_job_ids.end(), jobRequestsRoot[index]["job_id"].asString());

      // Check if already completed
      if (it_completed != completed_job_ids.end())
      {
        int distance = it_completed - completed_job_ids.begin();
        // cout << "Number of Completed Jobs " << completed_job_ids.size() << "\n";
      }
      else
      {

        if (it_high != high_priority_job_ids.end())
        {
          int distance = it_high - high_priority_job_ids.begin();
          // cout << "Distance high " << distance << "\n";
        }
        else
        {
          if (jobRequestsRoot[index]["priority"].asString() == "High Priority")
          {
            auto high_it = locations.find(jobRequestsRoot[index]["source"].asString());
            // cout << "High Priority Job is " << it->first << " = " << it->second << endl;
            high_priority_goals.push_back(high_it->second);
            high_priority_job_ids.push_back(jobRequestsRoot[index]["job_id"].asString());
          }
        }

        if (it_low != low_priority_job_ids.end())
        {
          int distance = it_low - low_priority_job_ids.begin();
          // cout << "Distance low " << distance << "\n";
        }
        else
        {
          if (jobRequestsRoot[index]["priority"].asString() == "Low Priority")
          {
            auto low_it = locations.find(jobRequestsRoot[index]["source"].asString());
            low_priority_goals.push_back(low_it->second);
            low_priority_job_ids.push_back(jobRequestsRoot[index]["job_id"].asString());
          }
        }
      }
    }
    //  cout << job_update.data << endl;
    //  jobresp_pub.publish(job_update);
  }
}

//function to get distance
double get_dist(geometry_msgs::Point point_1, geometry_msgs::Point point_2);

int main(int argc, char **argv)
{

  ward_a.pose.position.x = 5.984;
  ward_a.pose.position.y = 12.910;

  ward_b.pose.position.x = 5.583;
  ward_b.pose.position.y = 11.380;

  home.pose.position.x = 1.205;
  home.pose.position.y = 12.322;

  lab.pose.position.x = 0.988;
  lab.pose.position.y = 13.492;

  locations["Ward A"] = ward_a;
  locations["Ward B"] = ward_b;
  locations["Lab"] = lab;

  high_priority_goals.push_back(ward_a);
  high_priority_goals.push_back(ward_b);
  low_priority_goals.push_back(ward_b);
  bool high_priority;

  

  ros::init(argc, argv, "karbot_2dnavgoals");
  ros::NodeHandle n;
  ros::NodeHandle a;
  ros::NodeHandle jobReqHandle;

  ros::NodeHandle jobRespHandle;
  ros::Publisher jobresp_pub = jobRespHandle.advertise<std_msgs::String>("/karbot/jobResponse", 0);

  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/goal", 1000);
  ros::Subscriber amcl_sub = a.subscribe("/amcl_pose", 1, amclCallback);
  ros::Subscriber jobreq_sub = jobReqHandle.subscribe("/karbot/jobRequest", 1000, jobRequestCallback);

  ros::Rate loop_rate(5);
  ros::Rate pub_rate(0.2);

  int index = 0;
  //Wait for AMCL node to publish pose before proceeding
  while (!first_pose_loaded)
  {
    ROS_WARN_ONCE("Waiting for AMCL");
    ros::spinOnce();
  }
  ROS_WARN_ONCE("AMCL received");

  /*while (!first_message_received)
  {
    ROS_WARN_ONCE("Waiting for comms with web interface");
    ros::spinOnce();
  }*/
  ros::Duration(10).sleep();

  while (ros::ok())
  {
   
      if(Goal.pose.position != lab.pose.position){
        if (high_priority_goals.size() != 0)
        {
          // Set goal point to top of queue
          Goal = high_priority_goals.front();
          high_priority=1;
          /*if (current_job_id != high_priority_job_ids.front())
          {
            current_job_id = high_priority_job_ids.front();
            // Send job update
            job_update.data = "{\"job_id\": " + current_job_id + ", " + "\"status\":\"In Progress\"}";
            cout << job_update.data << endl;
            jobresp_pub.publish(job_update);
          }*/
        }
        else if (low_priority_goals.size() != 0)
        {
          // Set goal point to top of queue
          Goal = low_priority_goals.front();
          high_priority=0;
          /*if (current_job_id != low_priority_job_ids.front())
          {
            current_job_id = low_priority_job_ids.front();
            // Send job update
            job_update.data = "{\"job_id\": " + current_job_id + ", " + "\"status\":\"In Progress\"}";
            cout << job_update.data << endl;
            jobresp_pub.publish(job_update);
          }*/
        }
        else
        {
          // Set goal point to home
          Goal = home;
        }
      }
      

    // Going to pickup from ward
    if ((get_dist(Goal.pose.position, position)) < 0.2 && (Goal.pose.position != lab.pose.position) && (Goal.pose.position != home.pose.position))
    {
      // check if already completed
      /*auto it_completed = std::find(completed_job_ids.begin(), completed_job_ids.end(), current_job_id);
      if (it_completed != completed_job_ids.end())
      {
        // If job already completed
      }
      else
      {
        // if job not already completed and update not sent

        // Check if its high priority job or low priority job
        // if its a high priority job?
        auto it_high = std::find(high_priority_job_ids.begin(), high_priority_job_ids.end(), current_job_id);
        if (it_high != high_priority_job_ids.end())
        {
          completed_job_ids.push_back(current_job_id);
          high_priority_job_ids.pop_front();
          high_priority_goals.pop_front();
          // Send job update

          job_update.data = "{\"job_id\": " + current_job_id + ", " + "\"status\":\"Picked up\"}";
          cout << job_update.data << endl;
          jobresp_pub.publish(job_update);
          Goal.pose.position = lab.pose.position;
        }

        // if its a low priority job?
        auto it_low = std::find(high_priority_job_ids.begin(), high_priority_job_ids.end(), current_job_id);
        if (it_low != low_priority_job_ids.end())
        {
          completed_job_ids.push_back(current_job_id);
          low_priority_job_ids.pop_front();
          low_priority_goals.pop_front();
          // Send job update

          job_update.data = "{\"job_id\": " + current_job_id + ", " + "\"status\":\"Picked Up\"}";
          cout << job_update.data << endl;
          jobresp_pub.publish(job_update);
          Goal.pose.position = lab.pose.position;
        }
      }*/

      // ELSE ITS AT HOME?
      Goal.pose.position = lab.pose.position;
    }

    // Going to Lab
    if ((get_dist(Goal.pose.position, position)) < 0.2 && (Goal.pose.position == lab.pose.position))
    {
      //remove from appropriate queue using flag
      if(high_priority){
        high_priority_goals.pop_front();
      } else {
        low_priority_goals.pop_front();
      }

      if (high_priority_goals.size() != 0)
        {
          Goal = high_priority_goals.front();
          high_priority=1;
        }
        else if (low_priority_goals.size() != 0)
        {
          // Set goal point to top of queue
          Goal = low_priority_goals.front();
          high_priority=0;
        }
        else
        {
          // Set goal point to home
          Goal = home;
        }
      // check if already completed
      /*auto it_completed = std::find(completed_job_ids.begin(), completed_job_ids.end(), current_job_id);
      if (it_completed != completed_job_ids.end())
      {
        // If job already completed
      }
      else
      {
        // if job not already completed and update not sent

        // Check if its high priority job or low priority job
        // if its a high priority job?
        auto it_high = std::find(high_priority_job_ids.begin(), high_priority_job_ids.end(), current_job_id);
        if (it_high != high_priority_job_ids.end())
        {

          // Send job update
          job_update.data = "{\"job_id\": " + current_job_id + ", " + "\"status\":\"Completed\"}";
          cout << job_update.data << endl;
          jobresp_pub.publish(job_update);

          // Go to next goal point --- Sorry for the repitition
          if (high_priority_job_ids.size() != 0)
          {
            // Set goal point to top of queue
            Goal = high_priority_goals.front();
            if (current_job_id != high_priority_job_ids.front())
            {
              current_job_id = high_priority_job_ids.front();
              // Send job update
              job_update.data = "{\"job_id\": " + current_job_id + ", " + "\"status\":\"In Progress\"}";
              cout << job_update.data << endl;
              jobresp_pub.publish(job_update);
            }
          }
          else if (low_priority_job_ids.size() != 0)
          {
            // Set goal point to top of queue
            Goal = low_priority_goals.front();
            if (current_job_id != low_priority_job_ids.front())
            {
              current_job_id = low_priority_job_ids.front();
              // Send job update
              job_update.data = "{\"job_id\": " + current_job_id + ", " + "\"status\":\"In Progress\"}";
              cout << job_update.data << endl;
              jobresp_pub.publish(job_update);
            }
          }
          else
          {
            // Set goal point to home
          }
        }

        // if its a low priority job?
        auto it_low = std::find(high_priority_job_ids.begin(), high_priority_job_ids.end(), current_job_id);
        if (it_low != low_priority_job_ids.end())
        {

          // Send job update
          job_update.data = "{\"job_id\": " + current_job_id + ", " + "\"status\":\"Completed\"}";
          cout << job_update.data << endl;
          jobresp_pub.publish(job_update);

          // Go to next goal point
          if (high_priority_job_ids.size() != 0)
          {
            // Set goal point to top of queue
            Goal = high_priority_goals.front();
            if (current_job_id != high_priority_job_ids.front())
            {
              current_job_id = high_priority_job_ids.front();
              // Send job update
              job_update.data = "{\"job_id\": " + current_job_id + ", " + "\"status\":\"In Progress\"}";
              cout << job_update.data << endl;
              jobresp_pub.publish(job_update);
            }
          }
          else if (low_priority_job_ids.size() != 0)
          {
            // Set goal point to top of queue
            Goal = low_priority_goals.front();
            if (current_job_id != low_priority_job_ids.front())
            {
              current_job_id = low_priority_job_ids.front();
              // Send job update
              job_update.data = "{\"job_id\": " + current_job_id + ", " + "\"status\":\"In Progress\"}";
              cout << job_update.data << endl;
              jobresp_pub.publish(job_update);
            }
          }
          else
          {
            // Set goal point to home
          }
        }
      }*/

      // ELSE ITS AT HOME?
    }

    goal_pub.publish(Goal);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return (0);
}

//function to get distance between 2 points
double get_dist(geometry_msgs::Point point_1, geometry_msgs::Point point_2)
{
  double x_diff;
  double y_diff;
  double dist;
  x_diff = point_1.x - point_2.x;
  y_diff = point_1.y - point_2.y;
  return (sqrt(pow(y_diff, 2) + pow(x_diff, 2)));
}
