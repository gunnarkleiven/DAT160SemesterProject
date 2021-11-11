#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point



class robot_leader():
    def __init__(self):
        rospy.init_node('RobotLeader', anonymous=False)
        rospy.loginfo("Robot Leader class initiated")

        # self.leader_pub = rospy.Publisher()


    #Given map position returns cartesian coordinates
    def mapToPosition(self, map_pos):
        y_cell = math.floor(map_pos/self.map.info.width)
        x_cell = map_pos - y_cell*self.map.info.width
        position = Point()
        position.x = self.map.info.origin.position.x + (x_cell)*self.map.info.resolution
        position.y = self.map.info.origin.position.y + (y_cell)*self.map.info.resolution
        return position

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    leader = robot_leader()
    leader.main()

