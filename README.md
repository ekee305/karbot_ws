# karbot_ws
A work space developed in order to develop and deploy Karbot robot within simulation.

The gmapping sub-system with Karbot simulated within AWS hospital environment can be run using command "roslaunch karbot_navigation karbot_slam.launch" after building workspace using "catkin_make" command as per standard ROS practise

Simulation of full navigation sub-system with Karbot simulated within AWS hospital environment can be run using command "roslaunch karbot_navigation karbot_navigation.launch" after building workspace using "catkin_make" command as per standard ROS practise

RRT source: "/src/path_planning/src/RRT_In_Hospital"
RRT* source: "/src/path_planning/src/RRT_Star_In_Hospital"
RT-RRT* source: "/src/path_planning/src/RT_RRT_Star_In_Hospital"
Motion controller source "src/motion_controller/src/motion_controller_with_amcl"

AMCL with tuned parameters source: "/src/karbot_navigation/launch/amcl.launch"
AMCL with default parameters source: "/src/karbot_localization/launch/amcl.launch"
To launch AMCL with tuned parameters in simulation, ensure line 16 in source "src/karbot_navigation/launch/karbot_navigation.launch" is written as follows:
"<include file="$(find karbot_navigation)/launch/amcl.launch"/>"
To launch AMCL with default parameters in simulation, ensure line 16 in source "src/karbot_navigation/launch/karbot_navigation.launch" is written as follows:
"<include file="$(find karbot_localization)/launch/amcl.launch"/>"
