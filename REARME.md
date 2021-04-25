# coordination_robot ROS package

* This package achieves coordination of robots for reaching goal positions by three robots. 
* There are two launch files: one launch file (`coordination_environment.launch`) can run all the environments and connection nodes, while the other launch file (`waypoint_send.launch waypoint_task:=NUMBER`) has a role of sending intended waypoints.
* It is robust to achieve diverse waypoints sent by ROS topic from rosparam passed via .yaml file.
* It also pops up Rviz node automatically and users can easily monitor the task performance.

## Requirements
* Tested on ROS kinetic (not sure about Melodic but it should work)
* custom_turtlebot3_description package installed
    * make sure to load that gazebo model by setting the gazebo model path.

## Build
* Clone the repository into workspace ,e.g.,`ros_workspace/src/`
* run the command `catkin_make`

## Run
* 1st terminal

    ```
    source ros_workspace/devel/setup.sh
    roslaunch coordination_robot_pkg coordination_environment.launch
    ```

    * Once run, you will see the pop-up of Gazebo and Rviz. 
    * It will start connection to the robot and TF broadcastor. There will be warning messages while TF being connected. If it pops up more than once, it is abnormal as the warning message shows. It will show only once as it gets connected correctly.

* 2nd terminal
    ```
    roslaunch coordination_robot_pkg waypoint_send.launch waypoint_task:=NUMBER
    ```
    * The NUMBER can be anything between 1 and 6. 
    * It will trigger the required tasks by the assignment, e.g., waypoint topic, waypoint allocate intention, service, action.
    * You can see the waypoints visualized at Rviz. 
    * Once that numbered task is done, this node will automatically shut down. Please give at least __5 seconds__ to initalize the robots in the first terminal. Then, launch this node again with another task number.   

## Attribution & Licensing

Materials substantially authored by Mingi Jeong. Copyright 2020 by Amazon.com, Inc. or its affiliates. Licensed MIT-0 - See LICENSE for further information
