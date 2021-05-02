
# Auction Robot ROS package

* This package achieves auctions by an auntioneer robot and goal tasks(waypoint positions) by three robots -- including the auctioneer -- as bidders. 
* There are two launch files: one launch file (`roslaunch auction_robot_pkg auction_environment.launch`) can run all the environments and TF connection nodes, while the other launch file (`roslaunch auction_robot_pkg waypoint_send.launch point_total:=NUMBER`) has a role of publishing randomly generated waypoints.
* It is robust to achieve diverse waypoints (positions and the number of waypoints) sent by ROS topic `/waypoints`. 
* It also pops up Rviz node automatically and users can easily monitor the task performance.


## Requirements
* Tested on ROS kinetic (not sure about Melodic but it should work)
* custom_turtlebot3_description package installed.
* `multipledispatch` library for function overloading in Python.
    ```
    pip install multipledispatch
    ```

## Build
* Clone the repository into workspace ,e.g.,`ros_workspace/src/`
* run the command `catkin_make`

## Run
* 1st terminal

    ```
    source ros_workspace/devel/setup.sh
    roslaunch auction_robot_pkg auction_environment.launch
    ```

    * Once run, you will see the pop-up of Gazebo and Rviz. 
    * It will start connection to the robot and TF broadcastor. There will be warning messages while TF being connected. If it pops up more than once, it is abnormal as the warning message shows. It will show only once as it gets connected correctly.

* 2nd terminal
    ```
    roslaunch auction_robot_pkg waypoint_send.launch point_total:=NUMBER
    ```
    * The NUMBER (default 3) can be anything from 1 to 50 (theoretically). However, I tested mainly between 3 and 7. I checked it is working with 20 points. See the report. 
    * It will trigger the required tasks by the assignment, e.g., waypoint topic, waypoint allocate intention, service, action.
    * You can see the waypoints visualized at Rviz. 
    * Once that numbered task is done, this node will automatically shut down. Please give at least __5 seconds__ to initalize the robots in the first terminal. Then, launch this node again with another point_total arugment.   

