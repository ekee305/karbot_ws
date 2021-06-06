# karbot_ws
A work space developed in order to develop and deploy Karbot robot in real-life and simulation.

-- KARBOT IN REAL LIFE --


-- KARBOT IN SIMULATION --
The gmapping sub-system with Karbot simulated within AWS hospital environment can be run using command
"$ roslaunch karbot_navigation karbot_slam.launch"
after building workspace using "catkin_make" command as per standard ROS practise

To move the Karbot within the hospital 
"$ roslaunch karbot_navigation karbot_teleop.launch"
to move the Karbot within the hosipital environment.

Once map is created successfully, open a new terminal and run command:
"$ rosrun map_server map_saver -f hospital_floorplan"

Simulation of full navigation sub-system with Karbot simulated within AWS hospital environment can be run using command 
"$ roslaunch karbot_navigation karbot_navigation.launch"

To launch AMCL with tuned parameters in simulation, ensure line 16 in source "src/karbot_navigation/launch/karbot_navigation_simulation.launch" is written as follows:
"<include file="$(find karbot_navigation)/launch/amcl.launch"/>"

To launch AMCL with default parameters in simulation, ensure line 16 in source "src/karbot_navigation/launch/karbot_navigation_simulation.launch" is written as follows:
"<include file="$(find karbot_localization)/launch/amcl.launch"/>"
