#!/usr/bin/env python

# import of python modules
from operator import itemgetter
import copy

# import of relevant libraries
import rospy # module for ROS APIs
import actionlib
from std_msgs.msg import Bool 
from geometry_msgs.msg import Polygon, PolygonStamped, Point32

# custome modules
import aux_function
from auction_robot_super import AuctionRobot, DEFAULT_DESTINATION_ACTION, DEFAULT_REGISTER_SERVICE, DEFAULT_WP_ALLOCATE_INTENTION_TOPIC, DEFAULT_FLUSH_OUT_TOPIC, FLUSH_OUT_TIME, DEFAULT_WP_TOPIC, DEFAULT_WORLD_FRAME, DEFAULT_AUCTION_SERVICE
from auction_robot_pkg.msg import Waypoint_init, array1d
from auction_robot_pkg.srv import ServiceRegistration, ServiceRegistrationResponse, ServiceAuctionResult
from auction_robot_pkg.msg import Coordination_DestinationAction, Coordination_DestinationGoal, Coordination_DestinationFeedback, Coordination_DestinationResult


class AuctionRobot_auctioneer(AuctionRobot):
    def __init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw):
        """ Constructor leader robot as inheritance of Superclass"""
        AuctionRobot.__init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw)
        self.leader = True # make leader flag as true
        self._wp_msg = None # if receing waypoint topic, it saves here
        self.member_register = dict() # key: robot name / value: robot global position
        self.wp_allocate_for_robots = dict()  # key: robot name / value: allocated waypoint

        # action related
        self.action_client = dict()  # key: robot name / value: action client for that specific key robot
        self.action_achieved = dict()  # key: robot name / value: boolean for action goal completed
        self.allocate_state = False # False: not allocate / True: allocated

        # auction related
        self.target_wps = dict()  # saver of target waypoint as hashmap (key is just numbering)
        self.target_wps_tmp = dict()
        self.total_task = None # saving total task (# of waypoints)
        self.auction_finish = False
        self.round = 1
        self.wp_auction_allocate_for_robots = dict() # key: robot name / value: allocated waypoint
        self.auction_srv_status = dict() # auction service status for each robot

        # setting up publishers/subscribers
        self._wp_sub = rospy.Subscriber(DEFAULT_WP_TOPIC, PolygonStamped, self._wp_call_back, queue_size=1)
        self._wp_allocate_intention_pub = rospy.Publisher(DEFAULT_WP_ALLOCATE_INTENTION_TOPIC, Waypoint_init, queue_size=1)
        self._flush_out_pub = rospy.Publisher(DEFAULT_FLUSH_OUT_TOPIC, Bool, queue_size=10)

        # define server for register (bidding) service (including the auctioneer robot, as it bids, too)
        for i in range(self.robot_total):
            rospy.Service("tb3_{}".format(str(i)) + "/" + DEFAULT_REGISTER_SERVICE, ServiceRegistration, self._bid_srv_response)
            self.member_register[i] = None # for the purpose or registering robot with distance cost
            self.wp_auction_allocate_for_robots[i] = list() # auction wp allocation for a specific robot (list will be appended)
            self.auction_srv_status[i] = False

        # define action client for action 
        # this leader as centralized client to spread to servers
        for k in range(self.robot_total):
            # own robot as a leader inclusive
            self.action_client[k] = actionlib.SimpleActionClient("tb3_{}".format(str(k)) + "/" + DEFAULT_DESTINATION_ACTION, Coordination_DestinationAction)
            self.action_achieved[k] = False

    def _wp_call_back(self, msg):
        """
        waypoint callback function to subscribe from waypoint publisher node of format in Polygon
        """
        if msg is not None:
            self._wp_msg = msg
            self.wp_saver()
            # rospy.loginfo("Leader received waypoint lists")

    def wp_saver(self):
        """
        function to save list of waypoints received from "waypoint_send" node
        """
        for i in range(len(self._wp_msg.polygon.points)):
            self.target_wps[i] = [self._wp_msg.polygon.points[i].x, self._wp_msg.polygon.points[i].y]

        # update initial information related to auction
        if self.round == 1 and len(self.target_wps_tmp) == 0:
            # saving total number of tasks matching the number of waypoints
            self.total_task = len(self.target_wps)
            
            # each wp status (hashmap) specifying "not allocated yet"
            for i in range(len(self._wp_msg.polygon.points)):
                self.target_wps_tmp[i] = False 

    def _wp_allocate_intention_publish(self):
        """
        publish a custom message to other robots to initiate the coordinated behavior
        """
        if (
            self._wp_msg is not None 
            and not self.allocate_state
            ):

            # intention message initialization and update
            intention_msg = Waypoint_init()
            intention_msg.header.stamp = rospy.Time.now()
            intention_msg.header.frame_id = DEFAULT_WORLD_FRAME

            intention_msg.leader_name = "tb3_{}".format(str(self.robot_name))
            intention_msg.text_info = "Auctioneer received waypoint command! Trying to open auction..."
            intention_msg.auction_round = self.round
            intention_msg.waypoints = self._wp_msg.polygon
            intention_msg.allocation_status = [bool_value for bool_value in self.target_wps_tmp.values()]

            self._wp_allocate_intention_pub.publish(intention_msg)
            # rospy.loginfo("{} Round waypoint intention allocation message".format(self.round))

    def _bid_srv_response(self, request):
        """
        auctioneer's service response as for the request sent by bidder robot client
        i.e., register bidder robot as a team member
        """
        # leader is receiving a request from a team member who is supposed to send 
        if request.robot_name in self.member_register.keys():
            #  leader's member pose info not saved but the request msg contains it from the member
            if request.distance_cost is not None and self.member_register[request.robot_name] is None:
                # retrived the pose of the robot (its local frame)
                self.member_register[request.robot_name] = request.distance_cost
                
                return ServiceRegistrationResponse(True)
           
            else:
                return ServiceRegistrationResponse(False)

    def auction_progress(self):
        # condition
        if (
            not self.allocate_state 
            and len(self.member_register) == self.robot_total  
            and not aux_function.none_in_dict(self.member_register) 
            and self.round <= self.total_task
            ):
            # print message key: robot number / value: distance cost to each wp
            rospy.loginfo("Bidding registered: {}".format(self.member_register))

            # 1) find minimum cost wp and robot
            # min cost among entire waypoints by all the robots
            min_value = min(cost for robot_name, distance_cost in self.member_register.items() for cost in distance_cost)
            # robot name and waypoint task number (i) to be allocated
            min_result = [
                (robot_name, i) for robot_name, distance_cost in self.member_register.items() 
                for i in range(len(distance_cost)) if distance_cost[i]==min_value and not self.target_wps_tmp[i]
                ] # checking not allocated by tmp dict
            
            # if there are duplicate costs, it assign the low index robot
            if len(min_result) > 1:
                min_result = min_result[0]

            # 2) information regarding auction win
            win_robot = min_result[0][0]
            win_wp_number = min_result[0][1]
            win_wp = self.target_wps[win_wp_number]

            rospy.loginfo("win robot: {} / win wp number: {}".format(win_robot, win_wp_number))

            # 3) update dynamic data as one auction round will be finished
            self.allocate_state = True # allocated state for global check
            self.wp_auction_allocate_for_robots[win_robot].append(win_wp)
            self.target_wps_tmp[win_wp_number] = True
            self.round += 1 # round increase
            
            print("wp_allocate_for_robots {}".format(self.wp_auction_allocate_for_robots))
            
            # 4) auction result service request sent to bidders
            self._auction_result_srv_request(win_robot, win_wp)

    def _auction_result_srv_request(self, win_robot, win_wp):
        # iteration
        for i in range(self.robot_total):
            rospy.wait_for_service("tb3_{}".format(str(i)) + "/" + DEFAULT_AUCTION_SERVICE)        

            try:
                # connection established by Proxy
                auction_result_srv_request = rospy.ServiceProxy("tb3_{}".format(str(i)) + "/" + DEFAULT_AUCTION_SERVICE, ServiceAuctionResult)
                
                # 1) win robot case
                if i == win_robot:
                    # server (bidder) response <== request by client (auctioneer)
                    server_response = auction_result_srv_request(i, win_wp)
                    status_robot = "WIN"

                # 2) loser robot case
                else:
                    empty_wp = list()
                    # server (bidder) response <== request by client (auctioneer)
                    server_response = auction_result_srv_request(i, empty_wp)
                    status_robot = "LOSE"

                # common result
                if server_response:
                    self.auction_srv_status[i] = True
                    rospy.logwarn("Auction {} result at round {} is successfully received by Robot {}".format(status_robot, self.round-1, str(i)))


            except rospy.ServiceException as e:
                rospy.logwarn("Robot {} Service call failed {}".format(str(i), e))


    def next_auction_round_auctioneer(self):
        # condition
        # 1) auction result service received by all the robots from the previous round
        # 2) to-go-round is not more than the required total round
        if (
            not aux_function.false_in_dict(self.auction_srv_status)
            and self.round <= self.total_task
            ):
            print("================================================================")
            rospy.loginfo("moving to the next round {}".format(self.round))
            # allocated state for global check
            self.allocate_state = False

            for i in range(self.robot_total):
                self.member_register[i] = None
                self.auction_srv_status[i] = False

        if self.round > self.total_task:
            self.auction_finish = True

    def goal_action_client(self):
        """
        fucntion to send action "goal" from client to server (including leader itself)
        """
        # leader allocated WPs to all the robots including itself
        if (
            self.allocate_state
            and not self.action_sent
            and self.auction_finish
            ):

            # action client initiate
            for k in range(self.robot_total):
                # robot does not have an auction win
                if len(self.wp_auction_allocate_for_robots[k]) == 0:
                    continue

                # robot has at least one auction win
                else:
                    
                    action_client = self.action_client[k]
                    action_client.wait_for_server()

                    goal = Coordination_DestinationGoal()

                    # list of list https://stackoverflow.com/questions/43130377/how-to-publish-subscribe-a-python-list-of-list-as-topic-in-ros
                    for j in range(len(self.wp_auction_allocate_for_robots[k])):
                        inner_array = array1d()
                        inner_array.array = self.wp_auction_allocate_for_robots[k][j]

                        # allocate wp for k th robot
                        # goal.goal_position = self.wp_auction_allocate_for_robots[k]
                        goal.goal_position.append(inner_array)

                    action_client.send_goal(goal, feedback_cb=self.action_feedback_cb)

            self.action_sent = True # flag to prevent this coming again


    def action_periodic_check(self):
        """
        function to check action completion result 
        """
        if self.action_sent:
            # result check
            for k in range(self.robot_total):
                # in case of a robot winning at least one auction
                if len(self.wp_auction_allocate_for_robots[k]) != 0:
                    if not self.action_achieved[k]:
                        action_client = self.action_client[k]
                        action_client.wait_for_result()
                        result = action_client.get_result()
                        # result = self.action_client[k].get_result()

                        if result:
                            self.action_achieved[k] = True
                            rospy.logwarn("Action success! from robot {}: {}".format(k, result))
                else:
                    self.action_achieved[k] = True
                    rospy.logwarn("Action no need as losing auction! from robot {}".format(k))

    def action_feedback_cb(self, msg):
        """
        call back function to monitor action feedback (distance to go)
        """
        return
        # rospy.loginfo("feedback received by robot {}: {}".format(msg.robot_name, msg.distance_to_go))


    def flush_out_after_task(self):
        """
        function to flush out after all robots reached their waypoints
        """
        # all true: action goal achieved by all robots
        if False not in self.action_achieved.values():
            flush_out_msg = Bool(True)

            start_time = rospy.get_rostime()
            while not rospy.is_shutdown():
                # flushing out message sent for 5 sec to ensure all initialized
                if rospy.get_rostime() - start_time >= rospy.Duration(FLUSH_OUT_TIME):
                    break

                # Publish message for flushing out
                self._flush_out_pub.publish(flush_out_msg)
                self.initialization_auctioneer()


    def initialization_auctioneer(self):
        """
        function to initialize properties (leader robot) after finishing one action goal
        """

        # 1) initialization as constructor
        self._wp_msg = None
        self.member_register = dict()
        self.wp_allocate_for_robots = dict()
        self.allocate_state = False # False: not allocate / True: allocated
        self.action_achieved = dict()

        # auction related
        self.target_wps = dict()
        self.target_wps_tmp = dict()
        self.total_task = None # saving total task (# of waypoints)
        self.auction_finish = False
        self.round = 1
        self.wp_auction_allocate_for_robots = dict() # key: robot name / value: allocated waypoint
        self.auction_srv_status = dict() # auction service status for each robot

        # 2) for service
        for i in range(self.robot_total):
            # if i == self.robot_name:
            #     continue # own robot as a leader skip
            self.member_register[i] = None
            self.wp_auction_allocate_for_robots[i] = list() # auction wp allocation for a specific robot (list will be appended)
            self.auction_srv_status[i] = False

        # 3) for action client (I know repetition, but just for explicit clarification)
        for k in range(self.robot_total):
            self.action_achieved[k] = False

        # 4) for action server
        self.action_sent = False
        self.register_result = False
        self.action_received = False

        # 5) dynamic properties
        self._odom_msg = None 

        # 6) auction related


    def spin(self):
        """
        general spin function for leader robot to loop around 
        Note: not all of them will do things as it is based on finite state machine
        """
        while not rospy.is_shutdown():
            # lookup transform
            self._look_up_transform()

            # 1. initial position publish to make TF broadcator can do static TF transform
            self._init_pose_publish()

            # 2. waypoint allocation intention publish
            self._wp_allocate_intention_publish()

            self.auction_progress()
            self.next_auction_round_auctioneer()

            # # 4-1. action client sent
            self.goal_action_client()

            # # 4-2. action server receive
            self.goal_action_server()

            # # 4-3. action periodic check
            self.action_periodic_check()

            # # 5. flushout
            self.flush_out_after_task()

            # publish rate
            self.rate.sleep()
