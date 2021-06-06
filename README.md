# karbot_ws
A work space developed in order to develop and deploy Karbot robot in real-life and simulation.

## KARBOT IN REAL LIFE 
The gmapping sub-system with Karbot can be run using command
"$ roslaunch karbot_slam karbot_slam.launch"
after building workspace using "catkin_make" command as per standard ROS practise

To move the Karbot run command
"$ roslaunch karbot_navigation karbot_teleop.launch"
to move the Karbot within the hosipital environment.

Once map is created successfully, open a new terminal and run command:
"$ rosrun map_server map_saver -f hospital_floorplan"

Simulation of full navigation sub-system with Karbot simulated within AWS hospital environment can be run using command 
"$ roslaunch karbot_navigation karbot_navigation.launch"

To give a goal point for the Karbot to navigate to, press the "2D Nav Goal" button in rviz, press and hold on the point where you want the Karbot to be, and drag the arrow in the direction where you want the Karbot to face, and finally release for the navigation to commence.

To use AMCL with tuned parameters in simulation, ensure line 16 in source "src/karbot_navigation/launch/karbot_navigation_simulation.launch" is written as follows:
"<include file="$(find karbot_navigation)/launch/amcl.launch"/>"

To use AMCL with default parameters in simulation, ensure line 16 in source "src/karbot_navigation/launch/karbot_navigation_simulation.launch" is written as follows:
"<include file="$(find karbot_localization)/launch/amcl.launch"/>"

-- KARBOT IN SIMULATION --
The gmapping sub-system with Karbot simulated within AWS hospital environment can be run using command
"$ roslaunch karbot_navigation karbot_slam.launch"
after building workspace using "catkin_make" command as per standard ROS practise

To move the Karbot within the hospital, open a new terminal and run command 
"$ roslaunch karbot_navigation karbot_teleop.launch"
to move the Karbot within the hosipital environment.

Once map is created successfully, open a new terminal and run command:
"$ rosrun map_server map_saver -f hospital_floorplan"

Simulation of full navigation sub-system with Karbot simulated within AWS hospital environment can be run using command 
"$ roslaunch karbot_navigation karbot_navigation.launch"

To use AMCL with tuned parameters in simulation, ensure line 16 in source "src/karbot_navigation/launch/karbot_navigation_simulation.launch" is written as follows:
"<include file="$(find karbot_navigation)/launch/amcl.launch"/>"

To use AMCL with default parameters in simulation, ensure line 16 in source "src/karbot_navigation/launch/karbot_navigation_simulation.launch" is written as follows:
"<include file="$(find karbot_localization)/launch/amcl.launch"/>"
