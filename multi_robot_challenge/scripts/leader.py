#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
import time
import util
from multi_robot_challenge.srv import RequestPoint, RequestPointResponse
from visualization_msgs.msg import Marker


class robot_leader():
    def __init__(self):
        rospy.init_node('RobotLeader', anonymous=False)
        rospy.loginfo("Robot Leader class initiated")

        self.displayed_first_map = False
        self.map = OccupancyGrid()
        self.all_interesting_map_positions = []

        # List to store all of the positions which are sent to the robots
        self.all_positions = []
        self.wall_positions = []
        self.unreachable_positions = []
        
         
        self.namespaces_robots = ["/tb3_0", "/tb3_1"]
        self.publishers = {}

        # Not used 
        # for namespace in self.namespaces_robots:
        #     self.publishers[namespace] = rospy.Publisher(namespace + "/leader_commands", Pose, queue_size=10)

        self.publishers["/map_available"] = rospy.Publisher("/map_available", Bool, queue_size=10)
        self.publishers["/targets"] = rospy.Publisher("/targets", Marker, queue_size=10)

        
        # Subscribers
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        rospy.Subscriber("/unreachable_positions", Point, self.unreachable_position_cb)

        # Service server
        self.robot_service = rospy.Service("/robot_service", RequestPoint, self.service_cb)


    def unreachable_position_cb(self, pos):
        rospy.loginfo("[leader] received an unreachable position")
        self.unreachable_positions.append(pos)


    def service_cb(self, request):
        rospy.loginfo("[leader] got request from {} with pos: [{}, {}]".format(request.robotname, request.robot_point.x, request.robot_point.y))

        r = rospy.Rate(1)
        while not self.displayed_first_map:
            # Wait for the first reading of the map to come in
            r.sleep()
            rospy.loginfo("[leader] sleeping")

        self.calculate_all_next_positions()
        rospy.loginfo("[leader] finished calculating all next positions")
        desired_point = self.find_closest_desired_point(request.robot_point)
        self.all_positions.append(desired_point)
        

        self.set_marker()

        res = RequestPointResponse()
        res.target_point = desired_point
        res.success = True
        res.nice_friendly_message = "Hello Robot! Here is a position for you to go to"
        return res


    #Given map position returns cartesian coordinates
    def mapToPosition(self, map_pos):
        y_cell = math.floor(map_pos/self.map.info.width)
        x_cell = map_pos - y_cell*self.map.info.width
        position = Point()
        position.x = self.map.info.origin.position.x + (x_cell)*self.map.info.resolution
        position.y = self.map.info.origin.position.y + (y_cell)*self.map.info.resolution
        return position



    def map_cb(self, map):
        # rospy.loginfo("[leader] received updated map")
        self.map = map

        if not self.displayed_first_map:
            # rospy.loginfo(str(map))
            # origin = self.map.info.origin.position
            # rospy.loginfo("origin=%s    mapToPosition(0)=%s" % (origin, self.mapToPosition(0)))
            # rospy.loginfo(len(map.data))
            rospy.loginfo("Received first map")
            self.displayed_first_map = True
        self.publishers["/map_available"].publish(True)



    def calculate_next_position(self):
        '''
        This is currently unused, as this only returns one (the first) position, and not all positions
        '''
        height = self.map.info.height
        width = self.map.info.width
        origin = self.map.info.origin.position

        desired_grid_pos = -1

        # width counter
        w_count = 0
        for x in range(len(self.map.data)):
            # if self.map.data[x] == 0:
            if self.map.data[x]  < 10:
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
        

    def calculate_all_next_positions(self):
        '''
        The idea behind this algorithm is to find an unexplored area on the map adjacent to an explored area,
        i.e., a point on the OccupancyGrid with a value of 0 next to a value of -1.
        Then, we send a robot there. 
        '''
        height = self.map.info.height
        width = self.map.info.width 
        origin = self.map.info.origin.position

        all_desired_positions = []

        time_start = time.time()
        # width counter
        w_count = 0
        for x in range(len(self.map.data)):
            if self.map.data[x] == 0:
                if self.check_adjacent("undiscovered", x, w_count, width, self.map.data) and self.check_adjacent("wall", x, w_count, width, self.map.data) and self.mapToPosition(x) not in self.unreachable_positions:
                    all_desired_positions.append(x)
            elif self.map.data[x] == 100:
                self.wall_positions.append(self.mapToPosition(x))

            if x is not 0 and x % width == 0:
                w_count += 1

        
        self.all_interesting_map_positions = [self.mapToPosition(x) for x in all_desired_positions]
        if len(self.all_interesting_map_positions) > 0:
            rospy.loginfo("[leader] found new positions. n = " + str(len(self.all_interesting_map_positions)) + " total n = " + str(len(self.map.data)))
        else:
            rospy.logwarn("[leader] WARNING could not find any POI")
        rospy.loginfo("[leader] this took {} seconds".format(time.time() - time_start))


    def check_adjacent(self, check_type, x, w_count, width, data):
        check_value = -1 if check_type == "undiscovered" else 100
        # We want to return True if we are looking for undiscovered pixels, but return False if we are looking for walls
        # By returning two opposite types of boolean values, I get to reuse this function for both checking for adjacent 
        # undiscovered locations as well as checking for adjacent walls. 
        return_type = True if check_type == "undiscovered" else False

        # Above
        if (x + 1) % width != 0 and self.map.data[x + 1]  == check_value:
            return return_type
        # Below
        elif (w_count * x) % width != 0 and data[x-1] == check_value:
            return return_type
        # Right
        elif w_count > 0 and data[x - width] == check_value:
            return return_type
        # Left
        elif x + width < len(data) and data[x + width] == check_value:
            return return_type

        return not return_type


    def find_closest_desired_point(self, robot_pos):
        # we use a list so we can sort the distances, but also keeps them stored in a dict for looking up the
        # value for different distances
        distances = []
        distances_lookup = {} 

        # This is currently not used, rather using the one from utils.py
        distance = lambda x1, y1, x2, y2: math.sqrt((x2 - x1)** 2 + (y2 - y1)**2)

        x1 = robot_pos.x
        y1 = robot_pos.y
        for desired_pos in self.all_interesting_map_positions:
            # tmp_dis = distance(x1, y1, desired_pos.x, desired_pos.y)
            tmp_dis = util.calcDistance(robot_pos, desired_pos)
            distances.append(tmp_dis)
            distances_lookup[tmp_dis] = desired_pos

        distances.sort()

        
        closest_wall = min([util.calcDistance(robot_pos, x) for x in self.wall_positions])
        # We set a minimum distance, so that the robot does not try go to a location too close,
        # for example if the robot is already standing right on top of it. 
        minimum_distance = 1.0
        idx_pointer = 0
        while idx_pointer < len(distances) and distances[idx_pointer] < minimum_distance and distances_lookup[distances[idx_pointer]] not in self.unreachable_positions:
            idx_pointer += 1
        shortest_distance = distances[idx_pointer]
        closest_pos = distances_lookup[shortest_distance]

        return closest_pos

    
    def set_marker(self):
        marker = Marker()
        marker.header.frame_id = self.map.header.frame_id
        marker.ns = "markers"
        marker.id = 0
        # marker.header.stamp = rospy.Time.now()
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1

        marker.color.a = 1.0    # alpha
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration()

        marker.points = self.all_positions
        self.publishers["/targets"].publish(marker)
        # rospy.loginfo("[leader] published marker")


    def start_robot_movements(self):
        pass
            

    def main(self):

        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == "__main__":
    leader = robot_leader()
    leader.main()


