import math
from geometry_msgs.msg import Point, Quaternion


class map_points():
    def __init__(self, _point, _map_id):
        self.pnt = Point()
        self.pnt = _point
        self.map_id = _map_id


class map_areas():

    def __init__(self):
        self.area = 0
        self.pnt_list = []

    def add_point(self, point):
        self.area += 1
        self.pnt_list.append(point)

class tree_nodes():

    def __init__(self, _node, _parent):
        self.parent = _parent
        self.node = _node
        self.children = []
        self.endpoint = False

    def addChild(self, newChild):
        self.children.append(newChild)


def calcOrientation(theta):
    orientation = Quaternion()
    v = (0,0,1)
    #theta = math.atan2(new_pos.y-pos.y,new_pos.x-pos.x)

    orientation.x = v[0]*math.sin(theta/2)
    orientation.y = v[1]*math.sin(theta/2)
    orientation.z = v[2]*math.sin(theta/2)
    orientation.w = math.cos(theta/2)

    return orientation

def calcDistance(pos1, pos2):
    dist = math.sqrt(math.pow((pos2.x - pos1.x),2) + math.pow((pos2.y - pos1.y),2))

    return dist
