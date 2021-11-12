#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import scripts.util
import actionlib


'''
This file was used as an attempt to to communicate between the leader and the robots with actions, where
the leader was the action client wanting action goals from the robots as the action servers
'''


class robot_leader():
    def __init__(self):
        rospy.init_node('RobotLeader', anonymous=False)
        rospy.loginfo("Robot Leader class initiated")

         
        # self.leader_pub = rospy.Publisher('/leader_commands', Pose, queue_size=10)
        self.namespaces_robots = ["/tb3_0", "/tb3_1"]
        self.publishers = {}

        # self.leader_pub = rospy.Publisher('leader_commands', Pose, queue_size=10)
        for namespace in self.namespaces_robots:
            self.publishers[namespace] = rospy.Publisher(namespace + "/leader_commands", Pose, queue_size=10)

    
        # Action clients for the two robots, which acts as the action servers
        self.robot0_client = actionlib.SimpleActionClient(self.namespaces_robots[0] + "_move_command", MoveBaseAction)
        self.robot0_client.wait_for_server()
        rospy.loginfo("after 0")
        self.robot1_client = actionlib.SimpleActionClient(self.namespaces_robots[1] + "_move_command", MoveBaseAction)
        self.robot1_client.wait_for_server()
        rospy.loginfo("after 1")
        
        # Subscribers
        rospy.Subscriber("/robot_information", String, self.receive_robot_information_cb)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb)


    def move_robot_done_cb(self, state, result):
        rospy.loginfo("[leader] Robot action finished")

    def move_robot_feedback_cd(self, feedback):
        rospy.loginfo("[leader feedback]" + str(feedback))


    #Given map position returns cartesian coor<dinates
    def mapToPosition(self, map_pos):
        y_cell = math.floor(map_pos/self.map.info.width)
        x_cell = map_pos - y_cell*self.map.info.width
        position = Point()
        position.x = self.map.info.origin.position.x + (x_cell)*self.map.info.resolution
        position.y = self.map.info.origin.position.y + (y_cell)*self.map.info.resolution
        return position

    def test_target_pose_with_topics(self):
        target = Pose()
        target.position.x = 1.0
        target.position.y = 5.0
        target.position.z = 0.0
        target.orientation = util.calcOrientation(0.0)

        # self.leader_pub.publish(target)
        # rospy.loginfo("Leader published to leader_commands")

        # Send to first robot
        first_namespace = self.namespaces_robots[0]
        second_namespace = self.namespaces_robots[1]

        self.publishers[first_namespace].publish(target)
        rospy.loginfo("Leader published to " + first_namespace)

        target.position.x = -1.0
        target.position.y = -5.0
        
        self.publishers[second_namespace].publish(target)
        rospy.loginfo("Leader published to " + second_namespace)


    def test_target_pose_with_actions(self):
        rospy.loginfo("Leader inside test actions")

        target = Pose()
        target.position.x = 1.0
        target.position.y = 5.0
        target.position.z = 0.0
        target.orientation = util.calcOrientation(0.0)

        goal0 = MoveBaseGoal()
        goal0.target_pose.header.frame_id = "tb3_0"
        goal0.target_pose.header.stamp = rospy.Time.now()
        goal0.target_pose.pose = target

        self.robot0_client.send_goal(goal0, done_cb=self.move_robot_done_cb, feedback_cb=self.move_robot_feedback_cd)
        rospy.loginfo("[leader] sent goal1")

        # Change the goal position
        target.position.x = -1.0
        target.position.y = -5.0

        goal1 = MoveBaseGoal()
        goal1.target_pose.header.frame_id = "tb3_1"
        goal1.target_pose.header.stamp = rospy.Time.now()
        goal1.target_pose.pose = target
        self.robot0_client.send_goal(goal1, done_cb=self.move_robot_done_cb, feedback_cb=self.move_robot_feedback_cd)
        rospy.loginfo("[leader] sent goal")


    def receive_robot_information_cb(self, msg):
        rospy.loginfo("Leader received information message: " + msg.data)

    def map_cb(self, map):
        rospy.loginfo("[leader] received updated map")
        

    def main(self):
        # self.test_target_pose_with_topics()
        self.test_target_pose_with_actions()

        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    leader = robot_leader()
    leader.main()


