#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point


class map_filter():
    def __init__(self):
        rospy.init_node('RobotController', anonymous=False)
        self.map_robot = OccupancyGrid()
        self.filtered_map = OccupancyGrid()

        rospy.Subscriber("map", OccupancyGrid, self.map_robot_callback)       
        while(len(self.map_robot.data) < 1):
            rospy.loginfo("waiting for map")
            rospy.sleep(1)
            
        if rospy.get_namespace() == "/tb3_0/":
            rospy.Subscriber("/tb3_1/odom", Odometry, self.curr_partner_pos_callback)
        elif rospy.get_namespace() == "/tb3_1/":
            rospy.Subscriber("/tb3_0/odom", Odometry, self.curr_partner_pos_callback)

        self.map_pub = rospy.Publisher('map_filtered', OccupancyGrid, queue_size=10)
        

    def map_robot_callback(self, data):
        self.map_robot = data

    def curr_partner_pos_callback(self, data):
        self.partner_pos = Point()
        self.partner_pos = data.pose.pose.position

    def main(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            
            self.filtered_map = self.map_robot
            self.change_map = list(self.map_robot.data)
            
            #ADD MAP FILTERING CODE HERE

            self.filtered_map.data = tuple(self.change_map)
            self.map_pub.publish(self.filtered_map)

            r.sleep()


        

if __name__ == "__main__":
    filter = map_filter()
    filter.main()