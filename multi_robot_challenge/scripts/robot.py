#!/usr/bin/env python

import rospy
import actionlib
import util
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped

class robot_controller():
    def __init__(self):
        rospy.init_node('robot', anonymous=False)
        rospy.loginfo("Robot controller class initiated")

        self.target = Pose()
        self.target.position.x = 0.0
        self.target.position.y = 0.0
        self.target.position.z = 0.0
        self.target.orientation = util.calcOrientation(0.0)

        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.move_base_client.wait_for_server()

    def move_base_done_cb(self,status,result):
        rospy.loginfo("robot finished navigation")

    def move_base_feedback_cb(self, feedback):
        rospy.loginfo("current status of robot")

    def test_target_pose(self):
        target = Pose()
        target.position.x = 1.0
        target.position.y = 5.0
        target.position.z = 0.0
        target.orientation = util.calcOrientation(0.0)
        
        return target



    def move_to_target(self):
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        # self.goal.target_pose.pose = self.target
        self.goal.target_pose.pose = self.test_target_pose()

        # Sends the goal to the action server.
        self.move_base_client.send_goal(self.goal, done_cb=self.move_base_done_cb, feedback_cb=self.move_base_feedback_cb)
        self.move_base_client.wait_for_result()



if __name__ == "__main__":
    robot = robot_controller()
    robot.move_to_target()
