# karbot_ws
A work space developed in order to develop and deploy Karbot robot within simulation.

Simulation of full navigation sub-system can be run using command "roslaunch karbot_navigation karbot_navigation.launch" after builing workspace using "catkin_make" command

RRT source: "/src/path_planning/src/RRT_In_Hospital"
RRT* source: "/src/path_planning/src/RRT_Star_In_Hospital"
RT-RRT* source: "/src/path_planning/src/RT_RRT_Star_In_Hospital"
Motion controller source "src/motion_controller/src/motion_controller_with_amcl"

AMCL with tuned parameters source: "/src/karbot_navigation/launch/amcl.launch"
AMCL with default parameters source: "/src/karbot_localization/launch/amcl.launch"
