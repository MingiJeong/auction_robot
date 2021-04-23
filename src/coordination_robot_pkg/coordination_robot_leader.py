#!/usr/bin/env python

# import of relevant libraries
import rospy # module for ROS APIs
from coordination_robot_super import CoordinationRobot
from coordination_robot_pkg.msg import Waypoint_init
from coordination_robot_pkg.srv import ServiceRegistration, ServiceRegistrationResponse
from geometry_msgs.msg import Polygon, Point32

# custome modules
import aux_function

# constant
DEFAULT_REGISTER_SERVICE = 'register_service'

# TODO initialization after one task is done
#  wp_msg, dict for pose

class CoordinationRobot_leader(CoordinationRobot):
    def __init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw):
        """ Constructor """
        CoordinationRobot.__init__(self, robot_name, robot_total, init_x, init_y, init_z, init_yaw)
        self._wp_msg = None
        self._intented_wp = None
        self.member_register = dict()
        self.wp_allocate_for_robots = dict()
        self.target_wps = dict()

        # setting up publishers/subscribers
        self._wp_sub = rospy.Subscriber("waypoints", Polygon, self._wp_call_back, queue_size=1)
        self._wp_allocate_intention_pub = rospy.Publisher("wp_allocate_intention", Waypoint_init, queue_size=1)

        # Define server for service
        for i in range(self.robot_total):
            if i == self.robot_name:
                continue # own robot as a leader skip
            rospy.Service("tb3_{}".format(str(i)) + "/" + DEFAULT_REGISTER_SERVICE, ServiceRegistration, self._team_register)
            self.member_register[i] = None

    def _wp_call_back(self, msg):
        """
        waypoint callback function to subscribe from waypoint publisher node of format in Polygon
        """
        if msg is not None:
            self._wp_msg = msg
            self.wp_saver()
            # rospy.loginfo("Leader received waypoint lists")

    def wp_saver(self):
        for i in range(len(self._wp_msg.points)):
            self.target_wps[i] = [self._wp_msg.points[i].x, self._wp_msg.points[i].y]
        
        # print("TARGET received", self.target_wps)

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

    def _team_register(self, request):
        """
        service response from the request sent by follower client
        """
        # leader is receiving a request from a team member who is supposed to send 
        if request.robot_name in self.member_register.keys():
            #  leader's member pose info not saved but the request msg contains it from the member
            if request.global_position is not None and self.member_register[request.robot_name] is None:
                # retrived the pose of the robot (its local frame)
                self.member_register[request.robot_name] = request.global_position
                
                print("REGISTER", self.member_register[request.robot_name])
                return ServiceRegistrationResponse(True)
           
            else:
                return ServiceRegistrationResponse(False)

    def waypoint_allocate(self):
        # valid procedure so far
        # 1) member register dict for followers by checking length
        # 2) all members global position retrieved saved in the dict
        if len(self.member_register) == self.robot_total - 1 and not aux_function.none_in_dict(self.member_register):
            #  robot can do transform with valid data
            if self._odom_msg is not None:
                # transform local psn to global
                self.transform_local_psn_wrt_global()
                # leader robot position(global) register
                self.member_register[self.robot_name] = [self.transformed_x_wrt_world, self.transformed_y_wrt_world]

                rospy.loginfo("allocate finish: {}".for self.member_register)


    def spin(self):
        while not rospy.is_shutdown():
            # lookup transform
            self._look_up_transform()

            # initial position publish to make TF broadcator can do static TF transform
            self._init_pose_publish()

            # waypoint allocation intention publish
            self._wp_allocate_intention_publish()

            # allocate waypoints
            self.waypoint_allocate()

            # publish rate
            self.rate.sleep()
