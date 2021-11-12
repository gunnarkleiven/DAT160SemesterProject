#!/usr/bin/env python

import rospy
import actionlib
import util
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
import time

class robot_controller():
    def __init__(self):
        rospy.init_node('robot', anonymous=False)
        self.robot_name = rospy.get_param("robot_name")
        rospy.loginfo("Robot controller class initiated, name = " + self.robot_name)


        # Subscriber
        # With "/" makes them subscribe to the topic /leader_commands without the namespaces
        # omitting the "/" makes then subscribe to the topic with namespaces, e.g. /tb3_0/leader_commander
        # rospy.Subscriber('/leader_commands', Pose, self.leader_command_callback)
        rospy.Subscriber('leader_commands', Pose, self.leader_command_callback)

        # Publishers
        self.leader_info_pub = rospy.Publisher("/robot_information", String, queue_size=10)

        # Action server, receives goals from the leader
        self.action_server = actionlib.SimpleActionServer(self.robot_name + "_move_command", MoveBaseAction, execute_cb=self.execute_move_command_cb, auto_start=False)
        self.action_server.start()

        self.target = Pose()
        self.target.position.x = 0.0
        self.target.position.y = 0.0
        self.target.position.z = 0.0
        self.target.orientation = util.calcOrientation(0.0)

        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.move_base_client.wait_for_server()

        rospy.loginfo("[" + self.robot_name + "] finished")


    def execute_move_command_cb(self, goal):
        self.action_server.publish_feedback("This is a test feedback from " + self.robot_name)
        time.sleep(3)
        result = MoveBaseAction()
        self.action_server.set_succeeded(result)
    

    def move_base_done_cb(self,status,result):
        rospy.loginfo("robot finished navigation")
        # Also alert the leader that it has finished navigating
        self.leader_info_pub.publish("robot finished navigation")

    def move_base_feedback_cb(self, feedback):
        # rospy.loginfo("current status of robot: moving to x:%s y:%s z:%s" % (self.target.position.x, self.target.position.y, self.target.position.z))
        pass

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
        self.goal.target_pose.pose = self.target
        # self.goal.target_pose.pose = self.test_target_pose()

        # Sends the goal to the action server.
        self.move_base_client.send_goal(self.goal, done_cb=self.move_base_done_cb, feedback_cb=self.move_base_feedback_cb)
        self.move_base_client.wait_for_result()

    
    def leader_command_callback(self, pose):
        rospy.loginfo('Received command from leader')
        self.target = pose
        self.move_to_target()


    def main(self):
        rospy.spin()



if __name__ == "__main__":
    robot = robot_controller()
    # robot.move_to_target()
    robot.main()

