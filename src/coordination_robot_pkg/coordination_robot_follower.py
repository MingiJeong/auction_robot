#!/usr/bin/env python

# import of relevant libraries
import rospy # module for ROS APIs
from geometry_msgs.msg import Point32

# custom modules
from coordination_robot_super import CoordinationRobot
from coordination_robot_pkg.srv import ServiceRegistration
from coordination_robot_pkg.msg import Waypoint_init

# TODO topic name as variable
DEFAULT_REGISTER_SERVICE = 'register_service'

class CoordinationRobot_follower(CoordinationRobot):
    def __init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw):
        """ Constructor """
        CoordinationRobot.__init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw)
        self.leader_intention = False
        self.register_result = False
        
        # setting up publishers/subscribers
        self._wp_allocate_intention_sub = rospy.Subscriber("wp_allocate_intention", Waypoint_init, self._wp_allocate_intention_call_back)

    def _wp_allocate_intention_call_back(self, msg):
        # TODO once receive the state changes as new message
        # TODO when finish the task action ==> states goes back to original
        """
        receive a custom message type from the leader to indicate that the leader wants to initiatean allocation
        """
        if msg is not None:
            # received the leader's intention
            self.leader_intention = True

            # let's start register my robot then
            self._register_srv_request()

    def _register_srv_request(self):
        """
        follower sends a request for registration as a member to leader
        """
        # check that follower received the leader's intention and not yet it is registered
        if self.leader_intention and not self.register_result:
            # service wait, i.e., when it only exists
            rospy.wait_for_service("tb3_{}".format(str(self.robot_name)) + "/" + DEFAULT_REGISTER_SERVICE)
            
            try:
                # connection established by Proxy
                register_srv_request = rospy.ServiceProxy("tb3_{}".format(str(self.robot_name)) + "/" + DEFAULT_REGISTER_SERVICE, ServiceRegistration)
                
                #  robot can do transform with valid data
                if self._odom_msg is not None:
                    # transform local psn to global
                    self.transform_local_psn_wrt_global()
                
                transformed_psn = [self.transformed_x_wrt_world, self.transformed_y_wrt_world]

                # server (leader) response as per request by sender
                server_response = register_srv_request(self.robot_name, transformed_psn)
                
                if server_response: # register success (True) by the leader
                    self.register_result = server_response
                    rospy.loginfo("Robot {} is registered as a team by the leader robot".format(str(self.robot_name)))
                # else: it will go until it register since self.register_result still False
                
            except rospy.ServiceException as e:
                rospy.loginfo("Robot {} Service call failed {}".format(str(self.robot_name), e))

    def spin(self):
        while not rospy.is_shutdown():
            # lookup transform
            self._look_up_transform()

            # initial position publish to make TF broadcator can do static TF transform
            self._init_pose_publish()

            # publish rate
            self.rate.sleep()