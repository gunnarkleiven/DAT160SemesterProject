#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point


class map_merging():
    def __init__(self):
        rospy.init_node('MapMerging', anonymous=False)

        rospy.Subscriber('map_filtered', OccupancyGrid, self.map_filtered_callback)


    
    def map_filtered_callback(self, data):
        # TODO 
        pass


    def main(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():

            r.sleep()


        

if __name__ == "__main__":
    map_merging = map_merging()
    map_merging.main()