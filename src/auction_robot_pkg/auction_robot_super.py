#!/usr/bin/env python

# import of python modules
import math
import numpy as np

# import of relevant libraries
import rospy # module for ROS APIs
import tf
import actionlib
from geometry_msgs.msg import PoseWithCovariance, Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# import custom modules
from auction_robot_pkg.msg import Coordination_DestinationAction, Coordination_DestinationGoal, Coordination_DestinationFeedback, Coordination_DestinationResult
import aux_function

# CONSTANT
DEFAULT_POSE_TOPIC = 'initialpose' 
DEFAULT_ODOM_TOPIC = 'odom'
DEFAULT_WP_TOPIC = 'waypoints'
DEFAULT_WP_ALLOCATE_INTENTION_TOPIC = "wp_allocate_intention"
DEFAULT_FLUSH_OUT_TOPIC = "flushout"
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'

DEFAULT_ODOM_FRAME = 'odom'
DEFAULT_WORLD_FRAME = 'world'

DEFAULT_DESTINATION_ACTION = 'go_to_waypoint'
DEFAULT_REGISTER_SERVICE = 'register_service'

RATE = 10 # Hz
LOOK_UP_DELAY = 5 # sec
FLUSH_OUT_TIME = 5 # sec
LINEAR_VELOCITY = 0.2 # m/s
ANGULAR_VELOCITY = math.pi/10 # rad/s

class AuctionRobot():
    def __init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw):
        """ Constructor of Superclass"""
        self.robot_name = robot_name # own robot ID (int)
        self.robot_total = robot_total # total robot
        self.init_pose = [init_x, init_y, init_z, init_yaw] # initial robot pose passed as param
        self._odom_msg = None # keep updating odom msg to be saved
        self.linear_vel = LINEAR_VELOCITY
        self.angular_vel = ANGULAR_VELOCITY

        # setting up publishers/subscribers 
        self._cmd_pub = rospy.Publisher("tb3_{}".format(str(self.robot_name)) + "/" + DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        self._init_pose_pub = rospy.Publisher("tb3_{}".format(str(self.robot_name)) + "/" + DEFAULT_POSE_TOPIC, PoseWithCovariance, queue_size=1)
        self._odom_sub = rospy.Subscriber("tb3_{}".format(str(self.robot_name)) + "/" + DEFAULT_ODOM_TOPIC, Odometry, self._odom_call_back, queue_size=1)

        # transformation related
        self.g_R_o = None # Rotation matrix of odom frame w.r.t global frame
        self.g_T_o = None # transformation matrix of odom frame w.r.t. global frame
        self._odom_frame = "tb3_{}".format(str(self.robot_name)) + "/" + DEFAULT_ODOM_FRAME
        self._tf_listener = tf.TransformListener()
        self.transformed_x_wrt_world = None
        self.transformed_y_wrt_world = None

        # whether it is a leader or not flag
        self.leader = False

        # for action server on all robots
        self.action_sent = False  # true: after client send action goal
        self.action_destination = None # received destination position via action goal
        self.register_result = False # true: after client receives response from server that register a team member
        self.action_received = False # true: after server receives action goal
        
        # setting up action server (apply to all robots including the leader)
        self.action_server = actionlib.SimpleActionServer(
                "tb3_{}".format(str(self.robot_name)) + "/" + DEFAULT_DESTINATION_ACTION,
                 Coordination_DestinationAction, execute_cb=self.action_execute_cb, auto_start=False
                 )
        
        # topic publish rate
        self.rate = rospy.Rate(RATE)

    def _odom_call_back(self, msg):
        """
        keep updating the odom message; this will be necessary to transform position wrt odom into position wrt world
        """
        if msg is not None:
            self._odom_msg = msg

    def _look_up_transform(self):
        """
        look up transform by listener once the TF tree is established between the robot and world frame
        """
        try:
            self._tf_listener.waitForTransform(DEFAULT_WORLD_FRAME, self._odom_frame, rospy.Time(0), rospy.Duration(LOOK_UP_DELAY))
            (trans, rot) = self._tf_listener.lookupTransform(DEFAULT_WORLD_FRAME, self._odom_frame, rospy.Time(0))
            translation = tf.transformations.translation_matrix(trans)  # translation matrix
            rotation = tf.transformations.quaternion_matrix(rot)  # rotation matrix
            self.g_R_o = rotation
            self.g_T_o = np.dot(translation, rotation)

        except:
            rospy.logwarn("transformation connecting from robot {} to world frame".format(self.robot_name))
            rospy.logwarn("Robot {} is connected if not showing up this warn one more time ".format(self.robot_name))
    
    def transform_local_psn_wrt_global(self):
        """
        transformation local position (wrt odom) into global position (wrt world)
        """
        local_pos_x = self._odom_msg.pose.pose.position.x
        local_pos_y = self._odom_msg.pose.pose.position.y
        # robot psn on odom to be transformed wrt world
        transformed_point = np.dot(self.g_T_o, np.transpose(np.array([local_pos_x,local_pos_y,0,1]))) 
        self.transformed_x_wrt_world = transformed_point[0]
        self.transformed_y_wrt_world = transformed_point[1]

    def _init_pose_publish(self):
        """
        keep publishing initialpose message so that "tf_broadcastor" node can do static TF broadcast 
        for each robot frame vs. world frame
        """
        initial_pose_msg = PoseWithCovariance()
        initial_pose_msg.pose.position.x = self.init_pose[0]
        initial_pose_msg.pose.position.y = self.init_pose[1]
        initial_pose_msg.pose.position.z = self.init_pose[2]
        
        # euler to quaternion 
        # http://wiki.ros.org/tf2/Tutorials/Quaternions#Think_in_RPY_then_convert_to_quaternion
        q = quaternion_from_euler(0, 0, self.init_pose[3])
        initial_pose_msg.pose.orientation.x = q[0]
        initial_pose_msg.pose.orientation.y = q[1]
        initial_pose_msg.pose.orientation.z = q[2]
        initial_pose_msg.pose.orientation.w = q[3]

        self._init_pose_pub.publish(initial_pose_msg)

    def goal_action_server(self):
        """
        function to start goal action server
        http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29
        """
        # follower
        if not self.leader: 
            # the follower has been registered as a team member
            if self.register_result and not self.action_received:
                self.action_received = True # finite state machine
                self.action_server.start()
        # leader
        else: 
            if not self.action_received:
                self.action_received = True # finite state machine
                self.action_server.start()

    def action_execute_cb(self, goal):
        """
        action execute callback function by action server who receives action goal
        """
        # global variable to be used in self.moveforward function
        global feedback, success, initial_dist_to_go

        if self.action_received: # goal received
            self.action_destination = goal.goal_position

            # success flag unless it is interrupted
            success = True
            feedback = Coordination_DestinationFeedback()
            result = Coordination_DestinationResult()

            # rotation first
            self.rotate_in_place(self.first_heading_match())

            # proceed to the destination
            initial_dist_to_go = self.current_distance_to_go()
            self.move_forward(initial_dist_to_go)

            # end of moving forward and goal reached
            result.goal_reached = True

            if success:
                self.action_server.set_succeeded(result)

    def rotate_local_psn_wrt_global(self, vec_to_be_rotated):
        """
        function to rotate a vector based on rotation matrix of robot odom frame w.r.t world
        """
        rotated_point = np.dot(self.g_R_o, np.transpose(np.array([vec_to_be_rotated[0],vec_to_be_rotated[1],0,1]))) 
        reduced_2d = np.array([rotated_point[0], rotated_point[1]])
        return reduced_2d

    def current_distance_to_go(self):
        """
        function to check distance to go for the destination
        """
        # robot position update wrt world
        self.transform_local_psn_wrt_global()

        distnage_to_go = aux_function.distance_calculator(self.action_destination, [self.transformed_x_wrt_world, self.transformed_y_wrt_world])

        return distnage_to_go

    def first_heading_match(self):
        """
        function to match the robot heading with the 2nd vertex
        it returns +, - angle (for my double check) depending on check vector function (aux_function module)
        """

        # 1. get yaw angle of the robot when it reachsed the first vertex
        msg = self._odom_msg
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # 2. two vectors to find out between angles
        # base vector
        heading_vec_local = np.array([math.cos(yaw), math.sin(yaw)])
        heading_vec_global = self.rotate_local_psn_wrt_global(heading_vec_local)

        # toward goal vector (angle should be found between this and base vector)
        toward_goal_vec = np.array([self.action_destination[0] - self.transformed_x_wrt_world,
                                self.action_destination[1] - self.transformed_y_wrt_world]
                                )
        between_angle = aux_function.check_vector_angle(heading_vec_global, toward_goal_vec)
        rospy.loginfo("Robot {} angle to turn: {}".format(self.robot_name, math.degrees(between_angle)))

        return between_angle


    def move_forward(self, distance):
        """ 
        function to move forward for a given distance 
        """

        twist_msg = Twist()
        twist_msg.linear.x = self.linear_vel

        start_time = rospy.get_rostime()
        duration = rospy.Duration(distance/twist_msg.linear.x)

        while not rospy.is_shutdown():
            # Check if traveled of given distance based on time.
            if rospy.get_rostime() - start_time >= duration:
                break

            else:
                self._cmd_pub.publish(twist_msg)

                # ======= for feedback of action server ======= 
                if self.action_server.is_preempt_requested():
                    success = False
                    break
                
                feedback.robot_name = self.robot_name
                d_to_go = initial_dist_to_go - (rospy.get_rostime() - start_time).to_sec() * self.linear_vel
                
                feedback.distance_to_go = self.current_distance_to_go()
                self.action_server.publish_feedback(feedback)
                # ============================================ 

            # Sleep to keep the set publishing frequency.
            self.rate.sleep()

        # Traveled the required distance, stop.
        self.stop()


    def rotate_in_place(self, rotation_angle):
        """
        Rotate in place the robot of rotation_angle (rad) based on fixed velocity.
        depending on the rotation direction (clock vs counter-clock)
        """
        twist_msg = Twist()

        if rotation_angle > 0: # counter-clockwise True
            twist_msg.angular.z = self.angular_vel
        else: # counter-clockwise False, i.e., clockwise
            twist_msg.angular.z = - self.angular_vel
        
        duration = abs(rotation_angle) / abs(twist_msg.angular.z)
        start_time = rospy.get_rostime()

        while not rospy.is_shutdown():
            # Check if done
            if rospy.get_rostime() - start_time >= rospy.Duration(duration):
                break
                
            # Publish message.
            self._cmd_pub.publish(twist_msg)
            
            # Sleep to keep the set frequency.
            self.rate.sleep()

        # Rotated the required angle, stop.
        self.stop()

    def stop(self):
        """
        function to stop the robot by linear and angular velocity all zeros
        """
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)