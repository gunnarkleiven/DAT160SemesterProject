#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import util
import actionlib


class robot_leader():
    def __init__(self):
        rospy.init_node('RobotLeader', anonymous=False)
        rospy.loginfo("Robot Leader class initiated")

        self.displayed_first_map = False
         
        # self.leader_pub = rospy.Publisher('/leader_commands', Pose, queue_size=10)
        self.namespaces_robots = ["/tb3_0", "/tb3_1"]
        self.publishers = {}

        # self.leader_pub = rospy.Publisher('leader_commands', Pose, queue_size=10)
        for namespace in self.namespaces_robots:
            self.publishers[namespace] = rospy.Publisher(namespace + "/leader_commands", Pose, queue_size=10)

        
        # Subscribers
        rospy.Subscriber("/robot_information", String, self.receive_robot_information_cb)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb)



    #Given map position returns cartesian coordinates
    def mapToPosition(self, map_pos):
        y_cell = math.floor(map_pos/self.map.info.width)
        x_cell = map_pos - y_cell*self.map.info.width
        position = Point()
        position.x = self.map.info.origin.position.x + (x_cell)*self.map.info.resolution
        position.y = self.map.info.origin.position.y + (y_cell)*self.map.info.resolution
        return position

    def test_target_pose_with_topics(self):
        # TODO remove this
        time.sleep(3)
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


    def receive_robot_information_cb(self, msg):
        rospy.loginfo("Leader received information message: " + msg.data)

    def map_cb(self, map):
        rospy.loginfo("[leader] received updated map")
        self.map = map

        if not self.displayed_first_map:
            # rospy.loginfo(str(map))
            # origin = self.map.info.origin.position
            # rospy.loginfo("origin=%s    mapToPosition(0)=%s" % (origin, self.mapToPosition(0)))
            # rospy.loginfo(len(map.data))

            new_grid_pos = self.calculate_next_position()
            target = Pose()
            target.position = self.mapToPosition(new_grid_pos)
            target.orientation = util.calcOrientation(0.0)
            self.publishers["/tb3_0"].publish(target)
            rospy.loginfo("[leader] published new position to /tb3_0")

            self.displayed_first_map = True


    def calculate_next_position(self):
        '''
        The idea behind this algorithm is to find an unexplored area on the map adjacent to an explored area,
        i.e., a point on the OccupancyGrid with a value of 0 next to a value of -1.
        Then, we send a robot there. 
        '''
        height = self.map.info.height
        width = self.map.info.width
        origin = self.map.info.origin.position

        desired_grid_pos = -1

        # width counter
        w_count = 0
        for x in range(len(self.map.data)):
            if self.map.data[x] == 0:
                # Check for adjacent unknown area. Also check the bounds before accessing the array
                # Above 
                # if x + 1 < len(self.map.data) and self.map.data[x + 1] == -1:
                if (x + 1) % width is not 0 and self.map.data[x + 1] == -1:
                    desired_grid_pos = x
                    break
                # Below
                elif (w_count * x) % width is not 0 and self.map.data[x-1] == -1:
                # elif x > 0 and self.map.data[x-1] == -1:
                    desired_grid_pos = x
                    break
                # Right
                # elif w_count > 0 and self.map.data[(w_count - 1) * x] == -1:
                elif w_count > 0 and self.map.data[x - width] == -1:
                    desired_grid_pos = x
                    break
                # Left
                # elif w_count < height and self.map.data[w_count * x] == -1:
                elif w_count < height - 1 and self.map.data[x + width] == -1:
                    desired_grid_pos = x
                    break
            if x is not 0 and x % width == 0:
                w_count += 1
        

        if desired_grid_pos is not -1:
            rospy.loginfo("[leader] found POI at " + str(self.mapToPosition(desired_grid_pos)))
        else:
            rospy.logwarn("[leader] WARNING could not find POI")

        return desired_grid_pos



    def start_robot_movements(self):
        pass
    
        

    def main(self):
        # self.test_target_pose_with_topics()

        # Give the robot nodes some time to initiate
        time.sleep(3)

        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    leader = robot_leader()
    leader.main()


