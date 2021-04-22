#!/usr/bin/env python

# import of relevant libraries
import rospy # module for ROS APIs
from coordination_robot import CoordinationRobot


class CoordinationRobot_leader(CoordinationRobot):
    def __init__(self, robot_name, init_x, init_y, init_z, init_yaw):
        CoordinationRobot.__init__(self, robot_name, init_x, init_y, init_z, init_yaw)

    def spin(self):
        while not rospy.is_shutdown():
            self.init_pose_publish()
