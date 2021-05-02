#!/usr/bin/env python

# import of relevant libraries
import rospy # module for ROS APIs
import actionlib
from geometry_msgs.msg import Point32
from std_msgs.msg import Bool 

# custom modules
import aux_function
from auction_robot_super import AuctionRobot, DEFAULT_DESTINATION_ACTION, DEFAULT_REGISTER_SERVICE, DEFAULT_WP_ALLOCATE_INTENTION_TOPIC, DEFAULT_FLUSH_OUT_TOPIC
from auction_robot_pkg.srv import ServiceRegistration
from auction_robot_pkg.msg import Waypoint_init
from auction_robot_pkg.msg import Coordination_DestinationAction, Coordination_DestinationFeedback, Coordination_DestinationResult



class AuctionRobot_bidder(AuctionRobot):
    def __init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw):
        """ Constructor follower robot as inheritance of Superclass"""
        AuctionRobot.__init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw)
        
        # setting up publishers/subscribers
        # self._flush_out_sub = rospy.Subscriber(DEFAULT_FLUSH_OUT_TOPIC, Bool, self._flush_out_call_back, queue_size=10)

    """
    def _flush_out_call_back(self, msg):
        
        callback function to flush out follower robot
        
        self.initialization()

    def initialization_all(self):
        
        function to initialize properties (follower robot) after finishing one action goal
        

        # 1) task related properties
        self.auctioneer_intention = False
        self.action_sent = False
        self.action_destination = None
        self.register_result = False
        self.action_received = False

        # 2) dynamic properties
        self._odom_msg = None 
    """


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
            # self._register_srv_request()
            # self._bid_srv_request()

            # 3. action_server to receive 
            self.goal_action_server()

            # publish rate
            self.rate.sleep()