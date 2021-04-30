#!/usr/bin/env python

# import of relevant libraries
import rospy # module for ROS APIs
import actionlib
from geometry_msgs.msg import Point32
from std_msgs.msg import Bool 

# custom modules
from auction_robot_super import AuctionRobot, DEFAULT_DESTINATION_ACTION, DEFAULT_REGISTER_SERVICE, DEFAULT_WP_ALLOCATE_INTENTION_TOPIC, DEFAULT_FLUSH_OUT_TOPIC
from auction_robot_pkg.srv import ServiceRegistration
from auction_robot_pkg.msg import Waypoint_init
from auction_robot_pkg.msg import Coordination_DestinationAction, Coordination_DestinationFeedback, Coordination_DestinationResult



class AuctionRobot_bidder(AuctionRobot):
    def __init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw):
        """ Constructor follower robot as inheritance of Superclass"""
        AuctionRobot.__init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw)
        self.leader_intention = False # true: after receiving leader's waypoint intention msg

        # setting up publishers/subscribers
        self._wp_allocate_intention_sub = rospy.Subscriber(DEFAULT_WP_ALLOCATE_INTENTION_TOPIC, Waypoint_init, self._wp_allocate_intention_call_back)
        self._flush_out_sub = rospy.Subscriber(DEFAULT_FLUSH_OUT_TOPIC, Bool, self._flush_out_call_back, queue_size=10)

    def _wp_allocate_intention_call_back(self, msg):
        """
        receive a custom message type from the leader to indicate that the leader wants to initiatean allocation
        """
        if msg is not None:
            # received the leader's intention
            self.leader_intention = True

    def _flush_out_call_back(self, msg):
        """
        callback function to flush out follower robot
        """
        self.initialization()

    def initialization(self):
        """
        function to initialize properties (follower robot) after finishing one action goal
        """

        # 1) task related properties
        self.leader_intention = False
        self.action_sent = False
        self.action_destination = None
        self.register_result = False
        self.action_received = False

        # 2) dynamic properties
        self._odom_msg = None 


    def _register_srv_request(self):
        """
        follower sends a request (registration as a member) to leader
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
                    self.register_result = server_response # registered flag
                    rospy.logwarn("Robot {} is registered as a team by the leader robot".format(str(self.robot_name)))
                # else: it will go until it register since self.register_result still False
                
            except rospy.ServiceException as e:
                rospy.logwarn("Robot {} Service call failed {}".format(str(self.robot_name), e))

    def spin(self):
        """
        general spin function for follower robot to loop around 
        Note: not all of them will do things as it is based on finite state machine
        """
        while not rospy.is_shutdown():
            # lookup transform
            self._look_up_transform()

            # 1. initial position publish to make TF broadcator can do static TF transform
            self._init_pose_publish()

            # 2. register this robot service request to the leader
            self._register_srv_request()

            # 3. action_server to receive 
            self.goal_action_server()

            # publish rate
            self.rate.sleep()