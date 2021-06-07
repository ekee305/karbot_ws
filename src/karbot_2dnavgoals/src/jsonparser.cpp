//Authors: Ethan Kee, Kar Lau
//Date:17/05/2021
//Description: Simple program that can publish goal points on the /goal topic in a loop

#include "ros/ros.h"
#include <sstream>
#include <vector>
#include "std_msgs/String.h"
#include "jsoncpp.cpp"


using namespace std;

// for string delimiter
vector<string> split (string s, string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<string> res;

    while ((pos_end = s.find (delimiter, pos_start)) != string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

void jobRequestCallback(const std_msgs::String::ConstPtr& msg ) //might be quicker way of loading map by simply equating to data
{
    Json::Reader reader;
    Json::Reader reader1;
    Json::Value root;

    string json = msg->data.c_str();

    bool parseSuccess = reader.parse(json, root, false);
    

   

 if (parseSuccess)
  {
    const Json::Value resultValue = root["jobRequests"];
    cout << "Result is " << resultValue[1]["priority"]<< "\n";
  }



   // string str = msg->data.c_str();
   // string delimiter = "[{";
   // vector<string> v = split (str, delimiter);


   //for (auto i : v) cout << i << endl;
  // ROS_INFO(" I heard: [%s]", msg->data.c_str());
 //  ROS_INFO(" I heard: [%s]", v.size());
 // cout << "Hello" << endl;

}




int main(int argc, char **argv) {

  ros::init(argc, argv, "karbot_2dnavgoals");


  ros::NodeHandle a;


   //ROS_WARN(" Working??");

  ros::Subscriber jobreq_sub = a.subscribe("/karbot/jobRequest", 1000, jobRequestCallback);


  ROS_WARN_ONCE("Testing4");


   ros::spin();

   return(0);  
}
  










