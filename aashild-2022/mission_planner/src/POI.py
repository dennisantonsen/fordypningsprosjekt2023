from geometry_msgs.msg import Quaternion, Pose, Point

class POI():
    def __init__(self, identifier, pose: Pose):
        self.identifier = identifier
        self.pose = pose
        self.point = pose.position
        self.orientation = pose.orientation

    def __init__(self, identifier, point: Point, orientation: Quaternion):
        self.identifier = identifier
        self.point = point
        self.orientation = orientation
        self.pose = Pose(point, orientation)