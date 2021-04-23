#!/usr/bin/env python

import numpy as np

# import of relevant libraries
import rospy # module for ROS APIs
import tf
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

# CONSTANT
DEFAULT_POSE_TOPIC = 'initialpose' # TODO repetition
DEFAULT_ODOM_TOPIC = 'odom'
DEFAULT_ODOM_FRAME = 'odom'
DEFAULT_WORLD_FRAME = 'world'
RATE = 10
LOOK_UP_DELAY = 3

class CoordinationRobot():
    def __init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw):
        """ Constructor """
        self.robot_name = robot_name # own robot ID (int)
        self.robot_total = robot_total # total robot
        self.init_pose = [init_x, init_y, init_z, init_yaw]
        self._init_pose_pub = rospy.Publisher("tb3_{}".format(str(self.robot_name)) + "/" + DEFAULT_POSE_TOPIC, PoseWithCovariance, queue_size=1)
        self._odom_sub = rospy.Subscriber("tb3_{}".format(str(self.robot_name)) + "/" + DEFAULT_ODOM_TOPIC, Odometry, self._odom_call_back, queue_size=1)
        self._odom_msg = None # keep updating odom msg to be saved

        # transformation related
        self.g_T_o = None # transformation matrix of odom frame w.r.t. global frame
        self._odom_frame = "tb3_{}".format(str(self.robot_name)) + "/" + DEFAULT_ODOM_FRAME
        self._tf_listener = tf.TransformListener()
        self.transformed_x_wrt_world = None
        self.transformed_y_wrt_world = None

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
            self.g_T_o = np.dot(translation, rotation)
            # print("transformation working", self.robot_name)

        except:
            rospy.logwarn("transformation not working from robot {} to world frame".format(self.robot_name))

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
        # print("robot name {} publishing".format(self.robot_name))
        # self.rate.sleep()

    # def spin(self):
    #     while not rospy.is_shutdown():
    #         self.init_pose_publish()
