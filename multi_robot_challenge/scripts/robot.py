#!/usr/bin/env python

import rospy
import actionlib
import util
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped, Point, Twist
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from multi_robot_challenge.srv import GoPoint, RequestPoint, RequestPointRequest
from ar_track_alvar_msgs.msg import AlvarMarkers
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import SetBool
import time
import math

class robot_controller():
    def __init__(self):
        rospy.init_node('robot', anonymous=False)
        self.robot_name = rospy.get_param("robot_name")
        rospy.loginfo("Robot controller class initiated, name = " + self.robot_name)

        self.is_navigating = False
        self.map_available = False
        self.position_history = []
        self.timer = None
        self.prev_position_in_move_action = None
        self.backtracking = False

        # This is used to make the robot spin around every now and then 
        self.point_counter = 0
        self.timer_counter = 0

        # AR markers
        self.detected_markers = set()
        self.markers_in_sight = ['a']
        self.ar_id_lookup = {
            0: "fire",
            1: "fire",
            2: "human",
            3: "fire",
            4: "fire"
        }

        # Subscriber
        # With "/" makes them subscribe to the topic /leader_commands without the namespaces
        # omitting the "/" makes them subscribe to the topic with namespaces, e.g. /tb3_0/leader_commander
        rospy.Subscriber('leader_commands', Pose, self.leader_command_callback)
        rospy.Subscriber('odom', Odometry, self.odom_cb)
        rospy.Subscriber("/map_available", Bool, self.map_avail_cb)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_marker_cb)


        # Publishers
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.unreachable_pos_pub = rospy.Publisher("/unreachable_positions", Point, queue_size=10)


        # Service client(s)
        self.position_service_client = rospy.ServiceProxy("/robot_service", RequestPoint)
        rospy.wait_for_service("/robot_service")

        # Initial target
        self.target = Pose()
        self.target.position.x = 0.0
        self.target.position.y = 0.0
        self.target.position.z = 0.0
        self.target.orientation = util.calcOrientation(0.0)

        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.move_base_client.wait_for_server()

        # Publish to the leader that the robot is ready, and awaiting a position
        # self.leader_info_pub.publish(self.robot_name + " is ready!")
        rospy.loginfo("robot initiation finished")


    def ar_marker_cb(self, msg):
        for i in range(len(msg.markers)):
            marker_id = msg.markers[i].id
            if marker_id not in self.detected_markers and marker_id in self.ar_id_lookup.keys():
                rospy.logwarn("[{}] Found AR marker id:{} for a {}".format(self.robot_name, marker_id, self.ar_id_lookup[marker_id]))
                self.detected_markers.add(marker_id)

            # if marker_id in self.ar_id_lookup.keys():
            #     rospy.logwarn("Found AR marker id:{} for a {}".format(marker_id, self.ar_id_lookup[marker_id]))
            # else:
            #     rospy.loginfo("Found unknown AR marker: {}".format(marker_id))
    

    
    def map_avail_cb(self, msgBool):
        self.map_available = msgBool.data

    def map_cb(self, map):
        self.map = map


    def leader_command_callback(self, pose):
        '''
        Not in use, only used for testing purposes
        '''
        
        if not self.is_navigating:
            rospy.loginfo('Received command from leader')
            self.target = pose
            self.move_to_target()
        else:
            rospy.loginfo('is_navigating, ignoring command from leader')


    def odom_cb(self, new_odom):
        self.odom = new_odom
        self.position = new_odom.pose.pose.position


    def move_base_done_cb(self,status,result):
        rospy.loginfo("[{}] finished navigation".format(self.robot_name))
        if self.timer:
            self.timer.shutdown()
            self.timer = None
        # Also alert the leader that it has finished navigating
        # self.leader_info_pub.publish("robot finished navigation")
        self.is_navigating = False
        self.point_counter += 1

    def move_base_feedback_cb(self, feedback):

        # if self.backtracking: 
        #     rospy.loginfo("[{}] backtracking".format(self.robot_name))
        #     return
        # rospy.loginfo("current status of robot: moving to x:%s y:%s z:%s" % (self.target.position.x, self.target.position.y, self.target.position.z))

        position_from_feedback = feedback.base_position.pose.position
        if not self.prev_position_in_move_action:
            # If this is the first feedback from the action, set the prev position to this position and return without doing anything
            self.prev_position_in_move_action = position_from_feedback
            return
        
        # if self.timer:
        #     moved_distance = util.calcDistance(self.prev_position_in_move_action, position_from_feedback)

        threshold_without_timer = 0.3
        threshold_timer = 2.0
        # We make an assumption that it's fair to expect the robot to move at least 2 meters in the last 5 seconds. If not, it probably means it is stuck,
        # and it should do a small reset
        threshold = 1.0
        if not self.timer:
            self.timer = rospy.Timer(rospy.Duration(4), self.timer_cb, oneshot=True)
            self.prev_position_in_move_action = position_from_feedback
            return
        
        moved_distance = util.calcDistance(self.prev_position_in_move_action, position_from_feedback)
        if moved_distance > threshold:
            rospy.loginfo("[{}] moved far enough to stop timer".format(self.robot_name))
            self.timer.shutdown()
            self.timer = None
        # else:
        #     rospy.loginfo("[{}] timer still going".format(self.robot_name))

        # if not self.timer and moved_distance < threshold_without_timer:
        #     self.timer = rospy.Timer(rospy.Duration(5), self.timer_cb, oneshot=True)
        #     rospy.logwarn("[{}] timer initiated. Distance moved = {} ".format(self.robot_name, moved_distance))
        #     self.prev_position_in_move_action = position_from_feedback
        # elif moved_distance > threshold_timer:
        #     # There is a timer, so check if it has moved the expected distance before the timer runs out
        #     rospy.logwarn("[{}] moved back further away from the point, shutting down the timer".format(self.robot_name))
        #     self.timer.shutdown()
        #     self.timer = None

        


        # goal_position = self.goal.target_pose.pose.position
        # updated_distance = util.calcDistance(position_from_feedback, goal_position)

        # if updated_distance < 1.0 and not self.timer:
        #     self.timer = rospy.Timer(rospy.Duration(5), self.timer_cb, oneshot=True)
        #     rospy.logwarn("[{}] timer initiated. Distance = {} ".format(self.robot_name, updated_distance))
        # elif self.timer and updated_distance > 0.5:
        #     rospy.logwarn("[{}] moved back further away from the point, shutting down the timer".format(self.robot_name))
        #     self.timer.shutdown()
        #     self.timer = None


    def test_target_pose(self):
        target = Pose()
        target.position.x = 1.0
        target.position.y = 5.0
        target.position.z = 0.0 
        target.orientation = util.calcOrientation(0.0)
        
        return target


    def timer_cb(self, event):
        rospy.logwarn("[{}] timer went off! Cancelling move to target goal.".format(self.robot_name))
        self.move_base_client.cancel_goal()
        self.timer_counter += 1

        # We set the point where we tried to go to as unwanted position, as well as the adjacent positions
        # This could definately have been done better
        new_unreachable_positions = []
        up = self.target.position
        up.x += 1
        new_unreachable_positions.append(up)
        down = self.target.position
        down.x -= 1
        new_unreachable_positions.append(down)
        left = self.target.position
        up.y -= 1
        new_unreachable_positions.append(left)
        right = self.target.position
        up.y += 1
        new_unreachable_positions.append(right)

        for x in new_unreachable_positions:
            self.unreachable_pos_pub.publish(x)
        # self.unreachable_pos_pub.publish(self.target.position)
        self.is_navigating = False

        # Set the new target to be position we went to 2 times ago 
        # if len(self.position_history) > 1:
        #     self.target = self.position_history[-2]
        # else:
        #     # The length of the position history will always be > 0, because we manually add a position at the start
        #     self.target = self.position_history[-1]
        # self.target = self.home_position
        # self.backtracking = True
        # # self.backtracking_timer = rospy.Timer(rospy.Duration(5), self.backtracking_timer_cb, oneshot=True)
        # self.move_to_target()
        # self.backtracking = False


    def move_to_target(self):
        self.is_navigating = True
        self.position_history.append(self.target)
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = self.target
        # self.goal.target_pose.pose = self.test_target_pose()

        rospy.loginfo("[{} is navigating]".format(self.robot_name))
 
        # Sends the goal to the action server.
        self.move_base_client.send_goal(self.goal, done_cb=self.move_base_done_cb, feedback_cb=self.move_base_feedback_cb)
        self.move_base_client.wait_for_result()



    def do_full_rotation(self):
        '''
        Rotates the robot about 360 degrees. Spins for 8 seconds, so it's not accurately 1 spin (a little bit more)
        '''
        rospy.loginfo("[{}] started spinning".format(self.robot_name))
        twist_msg = Twist()
        twist_msg.angular.z = 1
        # self.cmd_vel_pub.publish(twist_msg)
        for _ in range(8):
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(1)
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        rospy.loginfo("[{}] finished spinning".format(self.robot_name))



    def main(self):
        # Kickstart everything by making the robots move slightly forward, to start the map updates
        # self.target = self.odom.pose.pose
        # self.target.position.x = self.target.position.x + 1
        # self.target.orientation = self.odom.pose.pose.orientation
        # rospy.loginfo("[{}] initially going to ({}, {})".format(self.robot_name, self.target.position.x, self.target.position.y))
        # self.move_to_target()
        r = rospy.Rate(1)

        while not self.map_available:
            r.sleep()

        self.home_position = self.odom.pose.pose

        
        self.target = self.odom.pose.pose
        self.target.position.x = self.target.position.x + 2.5
        # self.target.orientation = util.calcOrientation(math.pi / 2)
        rospy.loginfo("[{}] initially going to ({}, {})".format(self.robot_name, self.target.position.x, self.target.position.y))
        self.move_to_target()
        

        # Also initiate with a full spin
        self.do_full_rotation()

        
        while not rospy.is_shutdown():
            if self.is_navigating:
                r.sleep()
            else:
                if self.point_counter > 3:
                    self.point_counter = 0
                    self.do_full_rotation()
                
                # if self.timer_counter > 3:
                #     # reset to starting position
                #     self.timer_counter = 0
                #     self.target = self.home_position
                #     self.is_navigating = True
                #     self.move_to_target()
                # else:
                self.is_navigating = True
                request_newpos = RequestPointRequest()
                request_newpos.robot_point = self.position
                request_newpos.robotname = self.robot_name
                response = self.position_service_client(request_newpos)
                rospy.loginfo("[{}] received position ({}, {}), while being at position ({}, {})"
                    .format(self.robot_name, response.target_point.x, response.target_point.y, self.position.x, self.position.y))
                self.target = Pose()
                self.target.position = response.target_point
                self.target.orientation = util.calcOrientation(0.0)
                self.move_to_target()



    def main2_test(self):

        r = rospy.Rate(1)

        while not self.map_available:
            r.sleep()


        if self.robot_name == "tb3_1":
            self.target = self.odom.pose.pose
            self.target.position.x = 5.5
            self.target.position.y = -1.2
            rospy.loginfo("[{}] initially going to ({}, {})".format(self.robot_name, self.target.position.x, self.target.position.y))
            self.move_to_target()

            self.do_full_rotation()
            


        while not rospy.is_shutdown():
            r.sleep()


if __name__ == "__main__":
    robot = robot_controller()
    # robot.move_to_target()
    robot.main()
    # robot.main2_test()


