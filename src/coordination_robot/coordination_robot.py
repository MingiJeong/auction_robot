#!/usr/bin/env python

# import of relevant libraries
import rospy # module for ROS APIs
from geometry_msgs.msg import PoseWithCovariance
from tf.transformations import quaternion_from_euler

# CONSTANT
DEFAULT_POSE_TOPIC = 'initialpose' # TODO repetition
RATE = 10

class CoordinationRobot():
    def __init__(self, robot_name, init_x, init_y, init_z, init_yaw):
        """ Constructor """
        self.robot_name = robot_name # own robot ID (int)
        self.init_pose = [init_x, init_y, init_z, init_yaw]
        self._init_pose_pub = rospy.Publisher("tb3_{}".format(str(self.robot_name)) + "/" + DEFAULT_POSE_TOPIC, PoseWithCovariance, queue_size=1)
        self.rate = rospy.Rate(RATE)

    def init_pose_publish(self):
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
        self.rate.sleep()

    # def spin(self):
    #     while not rospy.is_shutdown():
    #         self.init_pose_publish()
