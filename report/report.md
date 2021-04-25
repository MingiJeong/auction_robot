# Coordination Robot ROS package
### CS269 Multi Robot Systems / 2021 Spring / PA-3
### Mingi Jeong / mingi.jeong.gr@dartmouth.edu

## 1. General
* This package achieves coordination of robots for reaching goal positions by three robots. 
* There are two launch files: one launch file (`roslaunch coordination_robot_pkg coordination_environment.launch`) can run all the environments and connection nodes, while the other launch file (`roslaunch coordination_robot_pkg waypoint_send.launch waypoint_task:=NUMBER`) has a role of sending intended waypoints.
* It is robust to achieve diverse waypoints sent by ROS topic from rosparam passed via .yaml file.
* It also pops up Rviz node automatically and users can easily monitor the task performance.

## 2. Method
1. Assumption
    * The _turtlebot3_waffle_pi_ model has been modified to use __encoder__ for odometry data. This is included in __custom_turtlebot3_description__ package.
    * The user starts robots with prior knowledge of initial positions and orientations of three robots with respect to the world frame. This is included in __coordination_environment.launch__. For example, the followings are first robot's argument for that pose. 
    * The user can modify this on the terminal while launching the nodes; however, it is advisable to modify in the launch file as there are total 3*3 = 9 arguments (except for z) to test spawn.    
    
        ```
        <arg name="first_tb3_x_pos" default="-3.0"/>
        <arg name="first_tb3_y_pos" default="0.0"/>
        <arg name="first_tb3_z_pos" default=" 0.0"/>
        <arg name="first_tb3_yaw"   default=" 1.57"/>
        ```

2. Start
    * Based on the above initial position arguments, (1) they are used for spawning robots in Gazebo for those poses, (2) they are passed to __coordination_robot__ node (`robot_load.launch`) to make connection of TF hierarchy. 
    * This is aligned with an real environment example where an user put three robots in indoor, e. g., in Sudikoff, set the reference frame, measure their poses, and pass them as arguments to do the real tests.
    * Specifically, in __coordination_robot__ node, the robot saves the initial pose from Rosparam and publishes `/initialpose` topic with message type `geometry_msgs/PoseWithCovarianceStamped` after transforming Euler to quaternion. 
    * Then, __tf_broadcastor__ node receives `/initialpose` topic and broadcast TF whenever (every call back) the message is received. 
    * The connection of TF hierarchy is given as follows. 

    <div style="text-align:center"><img src="../images/frames.png" width="500" height="100">
    </div>

3. Task-related
    1. Instantiation
    * A leader and followers are initiated by respective __coordination_robot__ nodes. Depending on arguments (leader flag), it will instantiate a relevant class despite the same reusable node code  
    * Another good implementation design is to use inheritances by super class (_coordination_robot_super.py_). All the common features are included in super class, while specific features from either leader (_coordination_robot_leader.py_) or follower (_coordination_robot_follower.py_) are implemented in the descendant class. 
    
    2. waypoint inputs
    * Note that after `coordination_environment.launch`, you will see robots in Gazebo and Rviz in addition to robot load and TF connection on the terminal with ros logwarn (visualized in yellow). 
    * Then, on another terminal, if you initiate `waypoint_send.launch`, it passes the waypoints saved in .yaml files. Arguments as `waypoint_task` can be selected from 1 to 6, but feel free to change any of them from the pre-made ones, if you want. 
    * The waypoints are sent via `waypoints` topic with message type `PolygonStamped`, which made it easier to visualize in Rviz. The example is as follows from the task number 6 with destination points in Cyan.
    <div style="text-align:center"><img src="../images/pa3-waypoint.png" width="250" height="200">
    </div>

4. Leader and Follower
    * In principle, I am using a finite state machine to check a certain state is met (e. g., a certain prior message is received so that a robot can head to the next step).
    1. Custom message for intention of coordination behavior
        * Once the leader receives `waypoints` topic (save in self._wp_msg), it sends the custom message via `wp_allocate_intention` topic to followers. 
        * The custom message (`Waypoint_init.msg`) includes (1) leader robot name (2) some string information (3) waypoints coordinated represented in a format `geometry_msgs/Polygon`, which again makes it easier to extract waypoint coordinates.
    2. Registration Service
        * Once the follower robot receives `wp_allocate_intention` topic, the follower (__client__) robot sends a request via Service proxy designated on that specific robot. 
        * The custom service `ServiceRegistration.srv` includes (1) follower robot name (2) global position transformed by TF listener implemented in Superclass (see self._look_up_transform, self.transform_local_psn_wrt_global).
        * The leader (__server__) receives the request, registers that specific follower robot, then, gives a response back with Bool.
        * The follower noticed it is registered and the result is logged with rospy.logwarn for the user to see.
    3. Allocation of waypoints
        * After the leader robot checks the condition (line 112), the leader allocates waypoints to robots (including the leader) based on the greedy approach (see self.waypoint_allocate).
        * For each waypoint, Euclidean distance is calculated and the waypoint is assigned to a robot. Then, that robot is deleted from the candidate lists to find another robot, which will be assigned to the next waypoint.
        * Using hash map data structure, I was able to efficiently handle this allocation (key: robot name, value: to-go waypoint)
    4. Action Goal
        * The custom action (`Coordination_Destination.action`) takes care of making robots reach waypoints. It includes (1) Goal position as a list of x,y coordinates (2) feedback as distance to go (Euclidean distance) (3) result as boolean to indicate the action goal is completed.
        * The leader has a role of __client__ connected to a respective robot (__server__). Note that the leader itself is a __server__, too. For this reason, the server is implemented in Superclass (line 134-178).
        * Once the __server__ receives the action goal, the robot (1) matches its heading towards its allocated destination (line 167 in Superclass) and (2) translate to the destination (line 171 in Superclass). During the action, the robot gives a feedback callback in terms of the remaining distance. It is printed out as rospy.loginfo.
        * Finally, when the robot reached the waypoint, the __client__ receives the result and the result is also logged as logwarn. 
    5. Flush out
        * There were some updates of class properties during the above steps. The main purpose is to block unnecessary repetition of previous steps. However, in order to perform the next waypoint task, they should be initialized.
        * Once the leader found out that all the robots reached the allocated waypoints, the leader publishes `flushout` topic for a certain period (5 sec in my case). Then, (1) the leader itself initializes the necessary properties, (2) the followers initializes, (3) waypoint_send node shuts down.
        * Note that I found out the server and client of 'service' and 'action' can be maintained, as they work as connection bridge. 
        * After the robots flushed out, if the user execute `waypoint_send.launch` with another task number, the robots resume the steps above: allocated to the new waypoints, go to the destination again.


## 3. Evaluation
Please see the additional material video.

1. Task performance
    * My code performs well with being well organized and commented.
    * It is robust in achieving diverse waypoints with initialized positions.
    * The code fully achieves the PA-3 criteria such as custom topic, action, service, TF broadcast, etc. 
    * You can see the visualization together with rviz running automatically. 

    <div style="text-align:center"><img src="../images/performing_task.png" width="300" height="180">
    </div>

2. Discussion
    * __Error of position__: I have observed errors as offsets from the intended destinations. There are several reasons as PA-1, but here are the main ones:
        * Dynamic and physics model of the robot: control of the robot is not always sharp as our intention due to friction, inertia, etc.
        * Odom based on encoder also brings about more errors. Odom is weak to drifting and odom data themselves include noise. Therefore, even when the robot is sharply at a starting position, I was able to observe the position slightly increasing as time elapsed.
        * This indicates that, in general, the more the robots move, the more errors arise.
        * One example is shown as the following two figures. _feedback_ on the terminal is based on Euclidean distance between the current transformed position with respect to the world and the goal position with respect to the world. On the other hand, _robot 2 d to go_ is calculated based on time passed * linear velocity. As a result, the latter seems to have not much error (simulated as a wheel encoder) while the actual global positions have larger offsets. 
        * Another interesting error is between Gazebo and Rviz. I assume that it is caused by some physics-related error such as friction, plus odometry. Specifically, the right two robots are aligned almost vertically in Rviz, while they are aligned diagonally in Gazebo.
    <div style="text-align:center"><img src="../images/action_success.png" width="300" height="140">
    </div>
    <div style="text-align:center"><img src="../images/offset.png" width="200" height="140">
    </div>
    <div style="text-align:center">error example on terminal and Rviz</div>
    <div style="text-align:center"><img src="../images/offset2.png" width="200" height="140">
    </div>
    <div style="text-align:center">error example between Gazebo and Rviz</div>

    * __Action server connection__ : I was about to re-instantiate action server connection during the flushout process. I realized that the original action server/client is still alive and it gave some error such as adding the same action server/client is not valid. Therefore, I used a finite state machine to block unnecessary procedures during the call back of the action, even if the action goal had been completed. 
