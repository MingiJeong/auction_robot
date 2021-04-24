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
from coordination_robot_super import CoordinationRobot, DEFAULT_DESTINATION_ACTION, DEFAULT_REGISTER_SERVICE, DEFAULT_WP_ALLOCATE_INTENTION_TOPIC, DEFAULT_FLUSH_OUT_TOPIC, FLUSH_OUT_TIME, DEFAULT_WP_TOPIC
from coordination_robot_pkg.msg import Waypoint_init
from coordination_robot_pkg.srv import ServiceRegistration, ServiceRegistrationResponse
from coordination_robot_pkg.msg import Coordination_DestinationAction, Coordination_DestinationGoal, Coordination_DestinationFeedback, Coordination_DestinationResult


class CoordinationRobot_leader(CoordinationRobot):
    def __init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw):
        """ Constructor leader robot as inheritance of Superclass"""
        CoordinationRobot.__init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw)
        self.leader = True # make leader flag as true
        self._wp_msg = None # if receing waypoint topic, it saves here
        self._intented_wp = None 
        self.member_register = dict() # key: robot name / value: robot global position
        self.wp_allocate_for_robots = dict()  # key: robot name / value: allocated waypoint
        self.target_wps = dict()  # saver of target waypoint as hashmap (key is just numbering)

        self.action_client = dict()  # key: robot name / value: action client for that specific key robot
        self.action_achieved = dict()  # key: robot name / value: boolean for action goal completed
        self.allocate_state = False # False: not allocate / True: allocated
        
        # setting up publishers/subscribers
        self._wp_sub = rospy.Subscriber(DEFAULT_WP_TOPIC, PolygonStamped, self._wp_call_back, queue_size=1)
        self._wp_allocate_intention_pub = rospy.Publisher(DEFAULT_WP_ALLOCATE_INTENTION_TOPIC, Waypoint_init, queue_size=1)
        self._flush_out_pub = rospy.Publisher(DEFAULT_FLUSH_OUT_TOPIC, Bool, queue_size=10)

        # define server for service
        for i in range(self.robot_total):
            if i == self.robot_name:
                continue # own robot as a leader skip
            rospy.Service("tb3_{}".format(str(i)) + "/" + DEFAULT_REGISTER_SERVICE, ServiceRegistration, self._team_register)
            self.member_register[i] = None

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

    def _wp_allocate_intention_publish(self):
        """
        publish a custom message to other robots to initiate the coordinated behavior
        """
        if self._wp_msg is not None:
            intention_msg = Waypoint_init()

            intention_msg.header.stamp = rospy.Time.now()
            intention_msg.header.frame_id = "/leader" + "/tb3_{}".format(str(self.robot_name))

            intention_msg.leader_name = "tb3_{}".format(str(self.robot_name))
            intention_msg.text_info = "Leader received waypoint command! Trying to allocate..."
            intention_msg.waypoints = self._wp_msg.polygon

            self._wp_allocate_intention_pub.publish(intention_msg)

    def _team_register(self, request):
        """
        service response from the request (register follower robot as team member) sent by follower client
        """
        # leader is receiving a request from a team member who is supposed to send 
        if request.robot_name in self.member_register.keys():
            #  leader's member pose info not saved but the request msg contains it from the member
            if request.global_position is not None and self.member_register[request.robot_name] is None:
                # retrived the pose of the robot (its local frame)
                self.member_register[request.robot_name] = request.global_position
                
                return ServiceRegistrationResponse(True)
           
            else:
                return ServiceRegistrationResponse(False)

    def waypoint_allocate(self):
        """
        function to allocate a waypoint to each robot based on greedy apporach
        each waypoint for each robot is saved in self.wp_allocate_for_robots dict
        """
        # valid condition check 
        # 1) waypoint not allocated yet
        # 2) member register dict for followers by checking length
        # 3) all members global position retrieved saved in the dict
        if not self.allocate_state and len(self.member_register) == self.robot_total - 1 and not aux_function.none_in_dict(self.member_register):
            #  robot can do transform with valid data
            if self._odom_msg is not None:

                # transform my local psn to global
                self.transform_local_psn_wrt_global()
                # leader robot position(global) register
                self.member_register[self.robot_name] = [self.transformed_x_wrt_world, self.transformed_y_wrt_world]

                # using deep copy in order not to affect self.member_register
                tmp_robot_psn_dict = copy.deepcopy(self.member_register)
                
                # iterate for distance comparison (greedy allocation)
                for wp_k, wp_psn in self.target_wps.items():
                    distance_finder = dict()
                    for robot_id, robot_psn in tmp_robot_psn_dict.items():
                        distance_finder[robot_id] = aux_function.distance_calculator(wp_psn, robot_psn)
                    
                    # find with greedy method
                    min_dist_robot, min_dist_psn = min(distance_finder.items(), key=itemgetter(1))
                    self.wp_allocate_for_robots[min_dist_robot] = wp_psn # allocate (key: robot name / value: destination)

                    # pop a robot with the shortest distance to the goal
                    tmp_robot_psn_dict.pop(min_dist_robot)

                # finite state machine for allocating
                rospy.logwarn("allocate finish: {}".format(self.wp_allocate_for_robots))
                self.allocate_state = True 

    def goal_action_client(self):
        """
        fucntion to send action "goal" from client to server (including leader itself)
        """
        # leader allocated WPs to all the robots including itself
        if self.allocate_state and not self.action_sent:

            # action client initiate
            for k in range(self.robot_total):

                action_client = self.action_client[k]
                action_client.wait_for_server()

                goal = Coordination_DestinationGoal()
                # allocate wp for k th robot
                goal.goal_position = self.wp_allocate_for_robots[k]

                action_client.send_goal(goal, feedback_cb=self.action_feedback_cb)

                # action_client.wait_for_result()
            
            self.action_sent = True # flag to prevent this coming again


    def action_periodic_check(self):
        """
        function to check action completion result 
        """
        if self.action_sent:
            # result check
            for k in range(self.robot_total):
                if not self.action_achieved[k]:
                    action_client = self.action_client[k]
                    action_client.wait_for_result()
                    result = self.action_client[k].get_result()

                    if result:
                        self.action_achieved[k] = True
                        rospy.logwarn("the action success! from robot {}: {}".format(k, result))


    def action_feedback_cb(self, msg):
        """
        call back function to monitor action feedback (distance to go)
        """
        rospy.loginfo("feedback received by robot {}: {}".format(msg.robot_name, msg.distance_to_go))


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
                self.initialization()


    def initialization(self):
        """
        function to initialize properties (leader robot) after finishing one action goal
        """

        # 1) initialization as constructor
        self._wp_msg = None
        # self.action_server = None
        self._intented_wp = None
        self.member_register = dict()
        self.wp_allocate_for_robots = dict()
        self.target_wps = dict()
        # self.action_client = dict()
        self.allocate_state = False # False: not allocate / True: allocated
        self.action_achieved = dict()

        # 2) for service
        for i in range(self.robot_total):
            if i == self.robot_name:
                continue # own robot as a leader skip
            self.member_register[i] = None

        # 3) for action client (I know repetition, but just for explicit clarification)
        for k in range(self.robot_total):
            self.action_achieved[k] = False

        # 4) for action server
        self.action_sent = False
        self.action_destination = None
        self.register_result = False
        self.action_received = False

        # 5) dynamic properties
        self._odom_msg = None 


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

            # 3. allocate waypoints
            self.waypoint_allocate()

            # 4-1. action client sent
            self.goal_action_client()

            # 4-2. action server receive
            self.goal_action_server()

            # 4-3. action periodic check
            self.action_periodic_check()

            # 5. flushout
            self.flush_out_after_task()

            # publish rate
            self.rate.sleep()
