
## Using the real Karbot 

#### Mapping
In order to map the hospital environemt, the gmapping sub-system is used. This can be run using command:\
`$ roslaunch karbot_slam karbot_slam.launch`\
after building workspace using "catkin_make" command as per standard ROS practise

To move the Karbot within the hosipital environment run command:\
`$ roslaunch karbot_navigation karbot_teleop.launch`

In order to save the map for Karbot to use run command:\
`$ rosrun map_server map_saver -f hospital_floorplan`

This map should then be place along within the map provided folder at:\
`karbot_ws/src/map_provider/maps`

#### Running the system
To use the system the intial postion of the robot must be set in the the AMCL package used for localisation. This is done by changing the default for the following arguments in the amcl.launch folder to match the robots intial location in the map:

  `<arg name="initial_pose_x" default="12"/>`\
  `<arg name="initial_pose_y" default="15"/>`\
  `<arg name="initial_pose_a" default="0"/>`
  
This launch file can be found at:\
`/karbot_ws/src/karbot_navigation/launch/amcl.launch`

In order to set the range of random sampling for the RT-RRT* algorithm, the upper and lower boundaries for the environment must be set. This is done by changing the following parameters in karbot_navigation.launch:\

   `<param name="upper_x" type="double" value="32" />`\
   `<param name="lower_x" type="double" value="7" />`\
   `<param name="upper_y" type="double" value="64" />`\
   `<param name="lower_y" type="double" value="7" />`
   
This launch file can be found at:\
`/karbot_ws/src/karbot_navigation/launch/karbot_navigation.launch`

After these parameters have been changed to match the environment, the navigation subsystem can be launched using the command:\
`$ roslaunch karbot_navigation karbot_navigation.launch`

Finally to give a goal point for the Karbot to navigate to, press the "2D Nav Goal" button in rviz, press and hold on the point where you want the Karbot to go to and then release. This will cause Karbot to navigate to this point.

#### Note on Job Scheduler
As the job scheduler is still in development, it has not been included in this version of the release, however it is being worked on and will be released in due course.

## KARBOT IN SIMULATION 
#### Mapping
In order to map the hospital environemt, the gmapping sub-system is used. This can be run using command:\
`$ roslaunch karbot_slam karbot_slam.launch`\
after building workspace using "catkin_make" command as per standard ROS practise

To move the Karbot within the hosipital environment run command:\
`$ roslaunch karbot_navigation karbot_teleop.launch`

In order to save the map for Karbot to use run command:\
`$ rosrun map_server map_saver -f hospital_floorplan`

This map should then be place along within the map provided folder at:\
`karbot_ws/src/map_provider/maps`

#### Running the system
To use the system the intial postion of the robot must be set in the the AMCL package used for localisation. This is done by changing the default for the following arguments in the amcl.launch folder to match the robots intial location in the map:

  `<arg name="initial_pose_x" default="12"/>`\
  `<arg name="initial_pose_y" default="15"/>`\
  `<arg name="initial_pose_a" default="0"/>`
  
This launch file can be found at:\
`/karbot_ws/src/karbot_navigation/launch/amcl.launch`

In order to set the range of random sampling for the RT-RRT* algorithm, the upper and lower boundaries for the environment must be set. This is done by changing the following parameters in karbot_navigation.launch:\

   `<param name="upper_x" type="double" value="32" />`\
   `<param name="lower_x" type="double" value="7" />`\
   `<param name="upper_y" type="double" value="64" />`\
   `<param name="lower_y" type="double" value="7" />`
   
This launch file can be found at:\
`/karbot_ws/src/karbot_navigation/launch/karbot_navigation.launch`

After these parameters have been changed to match the environment, the navigation subsystem can be launched using the command:\
`$ roslaunch karbot_navigation karbot_navigation.launch`

Finally to give a goal point for the Karbot to navigate to, press the "2D Nav Goal" button in rviz, press and hold on the point where you want the Karbot to go to and then release. This will cause Karbot to navigate to this point.

#### Note on Job Scheduler
As the job scheduler is still in development, it has not been included in this version of the release, however it is being worked on and will be released in due course.
