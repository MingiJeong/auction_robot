#!/usr/bin/env python

# import of relevant libraries
import rospy # module for ROS APIs
from coordination_robot_super import CoordinationRobot
from coordination_robot_pkg.msg import Waypoint_init
from coordination_robot_pkg.srv import ServiceRegistration, ServiceRegistrationResponse
from geometry_msgs.msg import Polygon

# constant
DEFAULT_REGISTER_SERVICE = 'register_service'

# TODO initialization after one task is done
#  wp_msg, dict for pose

class CoordinationRobot_leader(CoordinationRobot):
    def __init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw):
        CoordinationRobot.__init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw)
        self._wp_sub = rospy.Subscriber("waypoints", Polygon, self._wp_call_back, queue_size=1)
        self._wp_msg = None
        self._wp_allocate_intention_pub = rospy.Publisher("wp_allocate_intention", Waypoint_init, queue_size=1)
        self.member_register = dict()

        # Define SERVER
        for i in range(self.robot_total):
            if i == self.robot_name:
                continue # own robot as a leader skip
            rospy.Service("tb3_{}".format(str(i)) + "/" + DEFAULT_REGISTER_SERVICE, ServiceRegistration, self._team_register)
            self.member_register[i] = None

    def _team_register(self, request):
        # leader is receiving a request from a team member who is supposed to send 
        if request.robot_name in self.member_register.keys():
            #  leader's member pose info not saved but the request msg contains it from the member
            if request.global_position is not None and self.member_register[request.robot_name] is None:
                # retrived the pose of the robot (its local frame)
                self.member_register[self.robot_name] = request.global_position
                return ServiceRegistrationResponse(True)
            else:
                return ServiceRegistrationResponse(False)

    def _wp_call_back(self, msg):
        """
        waypoint callback function to subscribe from waypoint publisher node of format in Polygon
        """
        if msg is not None:
            self._wp_msg = msg
            # rospy.loginfo("Leader received waypoint lists")

    def _wp_allocate_intention_publish(self):
        """
        publish a custom message to other robots to initiate the coordinated behavior
        """
        # TODO some state that this is not yet reached and not allocated
        if self._wp_msg is not None:
            intention_msg = Waypoint_init()

            intention_msg.header.stamp = rospy.Time.now()
            intention_msg.header.frame_id = "/leader" + "/tb3_{}".format(str(self.robot_name))

            intention_msg.leader_name = "tb3_{}".format(str(self.robot_name))
            intention_msg.text_info = "Leader received waypoint command! Trying to allocate..."
            intention_msg.waypoints = self._wp_msg

            self._wp_allocate_intention_pub.publish(intention_msg)

    def spin(self):
        while not rospy.is_shutdown():
            # initial position publish to make TF broadcator can do static TF transform
            self._init_pose_publish()

            # waypoint allocation intention publish
            self._wp_allocate_intention_publish()

            # publish rate
            self.rate.sleep()
