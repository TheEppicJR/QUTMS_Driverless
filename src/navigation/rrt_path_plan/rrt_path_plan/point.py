# COPIED FROM ZED_CAMERA
# TODO: figure out a way to share python code among ROS packages
import numpy as np
from typing import Tuple
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point as PointMsg
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry
from math import expm1, sqrt, sin, cos
from transforms3d.euler import quat2mat

class Point:
    def __init__(self, x: float, y: float) -> None:
        self.x: float = x
        self.y: float = y

    def __add__(self, other: "Point") -> "Point":
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Point") -> "Point":
        return Point(self.x - other.x, self.y - other.y)

    def __truediv__(self, divisor: int) -> "Point":
        return Point(int(round(self.x/divisor)), int(round(self.y/divisor)))

    def __mul__(self, multiplier: int) -> "Point":
        return Point(self.x*multiplier, self.y*multiplier)

    def __len__(self):
        return 2

    def __getitem__(self, i):
        return (self.x, self.y)[i]
    
    def to_tuple(self) -> Tuple:
        return (self.x, self.y)

def ccw(A: Point, B: Point, C: Point) -> bool:
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)

class PointWithCov():
    def __init__(self,
        loc_x: float,
        loc_y: float,
        loc_z: float,
        loc_cov: np.array,
        color: int,
        header: Header,
        global_x: float = None,
        global_y: float = None,
        global_z: float = None,
        global_cov: np.array = None
        ) -> None:
        self.loc_x: float = loc_x
        self.loc_y: float = loc_y
        self.loc_z: float = loc_z
        self.loc_cov: np.array = loc_cov
        self.color: int = color
        self.isyellow: int = 0
        self.isblue: int = 0
        self.isorange: int = 0
        self.isbig: int = 0
        self.issmall: int = 0
        self.global_x: float = global_x
        self.global_y: float = global_y
        self.global_z: float = global_z
        self.global_cov: np.array = global_cov
        self.coords = (self.global_x, self.global_y)
        self.x, self.y = global_x, global_y
        self.header: Header = header
        self.header.frame_id = "map"
        self.nMeasurments: int = 0
        # have to start with 1 so we dont get a div by zero error
        self.ncMeasurments: int = 1
    
    def updatecolor(self, color):
        if color == 0:
            self.isblue += 1
            self.ncMeasurments += 1
        elif color == 1:
            self.isyellow += 1
            self.ncMeasurments += 1
        elif color == 2:
            self.isorange += 1
            self.isbig += 1
            self.ncMeasurments += 1
        elif color == 3:
            self.isorange += 1
            self.issmall += 1
            self.ncMeasurments += 1
        else:
            pass
        if self.isblue / self.ncMeasurments > 0.5:
            self.color = 0
        elif self.isyellow / self.ncMeasurments > 0.5:
            self.color = 1
        elif self.isorange / self.ncMeasurments > 0.5:
            if self.isbig / (self.isbig+self.issmall) > 0.5:
                self.color = 2
            else:
                self.color = 3
        else:
            self.color = 4

    def translate(self, odommsg: Odometry):
        poscov = np.array(odommsg.pose.covariance).reshape((6,6))
        coneLocation = np.array([self.loc_x, self.loc_y, self.loc_z])
        orientation_q = odommsg.pose.pose.orientation
        orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        rotation_matrix = quat2mat(orientation_list)
        new_cov = rotation_matrix @ self.loc_cov @ rotation_matrix.T
        self.global_cov = new_cov + poscov[0:3, 0:3]
        x = odommsg.pose.pose.position.x
        y = odommsg.pose.pose.position.y
        z = odommsg.pose.pose.position.z
        globalloc = rotation_matrix @ coneLocation
        self.global_x = x + globalloc[0]
        self.global_y = y + globalloc[1]
        self.global_z = z + globalloc[2]
        self.coords = (self.global_x, self.global_y)
        self.x, self.y = self.global_x, self.global_y

    def update(self, other:"PointWithCov"):
        m3, c3 = multivariate_multiply([self.global_x, self.global_y, self.global_z], self.global_cov, [other.global_x, other.global_y, other.global_z], other.global_cov)
        self.global_cov = c3
        self.global_x = m3[0]
        self.global_y = m3[1]
        self.global_z = m3[2]
        self.coords = (self.global_x, self.global_y)
        self.x, self.y = self.global_x, self.global_y
        self.nMeasurments += 1
        self.updatecolor(other.color)

    def covMax(self, lim) -> bool:
        return sqrt(self.global_cov[0,0]**2+self.global_cov[1,1]**2+self.global_cov[2,2]**2) < lim

    def covMin(self, lim) -> bool:
        return sqrt(self.global_cov[0,0]**2+self.global_cov[1,1]**2+self.global_cov[2,2]**2) > lim

    def inTwoSigma(self, other:"PointWithCov"):
        # get the vector between the points
        vector = [self.global_x-other.global_x, self.global_y-other.global_y, self.global_z-other.global_z]
        # get the normalized vector between the points
        dist = sqrt(vector[0]**2+vector[1]**2+vector[2]**2)+ 0.00001
        # calculate the distance fo 2 sigma in the direction of the normal vector
        selfTwosig = sqrt(((vector[0] / dist) * sqrt(abs(self.global_cov[0,0])))**2 + ((vector[1] / dist) * sqrt(abs(self.global_cov[1,1])))**2 + ((vector[2] / dist) * sqrt(abs(self.global_cov[2,2])))**2) * 2
        otherTwosig = sqrt(((vector[0] / dist) * sqrt(abs(other.global_cov[0,0])))**2 + ((vector[1] / dist) * sqrt(abs(other.global_cov[1,1])))**2 + ((vector[2] / dist) * sqrt(abs(other.global_cov[2,2])))**2) * 2
        # see if the distance is less than the sum of the two 2 sigma vectors
        return dist < selfTwosig + otherTwosig

    def inFourSigma(self, other:"PointWithCov"):
        # get the vector between the points
        vector = [self.global_x-other.global_x, self.global_y-other.global_y, self.global_z-other.global_z]
        # get the normalized vector between the points
        dist = sqrt(vector[0]**2+vector[1]**2+vector[2]**2)+ 0.00001
        # calculate the distance fo 4 sigma in the direction of the normal vector
        selfTwosig = sqrt(((vector[0] / dist) * sqrt(abs(self.global_cov[0,0])))**2 + ((vector[1] / dist) * sqrt(abs(self.global_cov[1,1])))**2 + ((vector[2] / dist) * sqrt(abs(self.global_cov[2,2])))**2) * 4
        otherTwosig = sqrt(((vector[0] / dist) * sqrt(abs(other.global_cov[0,0])))**2 + ((vector[1] / dist) * sqrt(abs(other.global_cov[1,1])))**2 + ((vector[2] / dist) * sqrt(abs(other.global_cov[2,2])))**2) * 4
        # see if the distance is less than the sum of the two 2 sigma vectors
        return dist < selfTwosig + otherTwosig


    def dist(self, other:"PointWithCov"):
        return sqrt((self.global_x-other.global_x)**2+(self.global_y-other.global_y)**2+(self.global_z-other.global_z)**2)

    # should add cone color to this
    def getMarker(self, id: int):
        return point_msg(self.global_x, self.global_y, self.global_z, id, self.header, self.color)

    def getCov(self, id: int, buffer: bool):
        # make a deformed sphere at 3 sigma of the variance in each axis (the diagnal elements of the covariance matrix are squared so we gotta sqrt)
        return cov_msg(self.global_x, self.global_y, self.global_z, id, self.header, 3*sqrt(abs(self.global_cov[0,0])), 3*sqrt(abs(self.global_cov[1,1])), 3*sqrt(abs(self.global_cov[2,2])), buffer)

    def __len__(self):
        return len(self.coords)

    def __getitem__(self, i):
        return self.coords[i]

    def __repr__(self):
        return 'PointWithCov({}, {}, {})'.format(self.coords[0], self.coords[1], self.color)

    def __eq__(self, other:"PointWithCov"):
        return self.global_x == other.global_x and self.global_y == other.global_y


class Edge():
    def __init__(self, p1: PointWithCov, p2: PointWithCov):
        self.p1: PointWithCov = p1
        self.p2: PointWithCov = p2
        self.x1 = self.p1.global_x
        self.y1 = self.p1.global_y
        self.x2 = self.p2.global_x
        self.y2 = self.p2.global_y
        self.intersection = None
        self.calledFor = False
        self.getColor()
        self.x, self.y = self.getMiddlePoint()


    def getColor(self):
        # 0 is blue, 1 is yellow, 2 is blue to orange, 3 is yellow to orange, 4 is orange to orange, 5 is unknown
        if self.p1.color == 0 and self.p2.color == 0:
            self.color = 0
        elif self.p1.color == 1 and self.p2.color == 1:
            self.color = 1
        elif (self.p1.color == 2 or self.p1.color == 3) and (self.p2.color == 2 or self.p2.color == 3):
            self.color = 4
        elif ((self.p1.color == 2 or self.p1.color == 3) and self.p2.color == 0) or ((self.p2.color == 2 or self.p2.color == 3) and self.p1.color == 0):
            self.color = 2
        elif ((self.p1.color == 2 or self.p1.color == 3) and self.p2.color == 1) or ((self.p2.color == 2 or self.p2.color == 3) and self.p1.color == 1):
            self.color = 3
        elif (self.p1.color < 2 and self.p2.color == 4) or (self.p2.color < 2 and self.p1.color == 4):
            self.color = 5
        else:
            self.color = 6
        #return self.color
        
    def getPointMsg(self):
        p1 = PointMsg()
        p1.x = self.x1
        p1.y = self.y1
        p1.z = 0.0
        p2 = PointMsg()
        p2.x = self.x2
        p2.y = self.y2
        p2.z = 0.0
        return p1, p2

    def intersect(self, A: Point, B: Point):
        c: Point = Point(self.x1, self.y1)
        d: Point = Point(self.x2, self.y2)
        return ccw(A,c,d) != ccw(B,c,d) and ccw(A,B,c) != ccw(A,B,d)

    def getMiddlePoint(self):
        return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

    def length(self):
        return sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

    def __getitem__(self, i):
        x, y = self.getMiddlePoint()
        return (x, y)[i]

    def __len__(self):
        return 2

    def getPartsLengthRatio(self):

        part1Length = sqrt((self.x1 - self.intersection[0]) ** 2 + (self.y1 - self.intersection[1]) ** 2)
        part2Length = sqrt((self.intersection[0] - self.x2) ** 2 + (self.intersection[1] - self.y2) ** 2)

        return max(part1Length, part2Length) / min(part1Length, part2Length)

    def __eq__(self, other: "Edge"):
        return (self.x1 == other.x1 and self.y1 == other.y1 and self.x2 == other.x2 and self.y2 == other.y2
             or self.x1 == other.x2 and self.y1 == other.y2 and self.x2 == other.x1 and self.y2 == other.y1)

    def __str__(self):
        return "Edge(" + str(round(self.x1, 2)) + "," + str(round(self.y1,2)) + "),(" + str(round(self.x2, 2)) + "," + str(round(self.y2,2)) + ")"

    def __repr__(self):
        return str(self)

class Triangle():
    def __init__(self, p1: PointWithCov, p2: PointWithCov, p3: PointWithCov, e1: Edge, e2: Edge, e3: Edge) -> None:
        self.p1: PointWithCov = p1
        self.p2: PointWithCov = p2
        self.p3: PointWithCov = p3
        self.e1: Edge = e1
        self.e2: Edge = e2
        self.e3: Edge = e3
        self.calcCentroid()

    def calcCentroid(self):
        self.x = (self.p1.global_x + self.p2.global_x + self.p3.global_x) / 3
        self.y = (self.p1.global_y + self.p2.global_y + self.p3.global_y) / 3
        self.z = (self.p1.global_z + self.p2.global_z + self.p3.global_z) / 3

    def getEdges(self):
        return (self.e1, self.e2, self.e3)

    def getPoints(self):
        return (self.p1, self.p2, self.p3)

    def __eq__(self, other: "Triangle"):
        return (self.x == other.x and self.y == other.y and self.z == other.z)

    def __str__(self):
        return "Triangle(" + str(round(self.x, 2)) + "," + str(round(self.y,2)) + "," + str(round(self.z,2)) + ")"

    def __repr__(self):
        return str(self)

    def __getitem__(self, i):
        return (self.x, self.y)[i]
    
    def __len__(self):
        return 2


def point_msg(
    x_coord: float, 
    y_coord: float, 
    z_coord: float,
    ID: int, 
    header: Header,
    color: int
) -> Marker: 
    """
    Creates a Marker object for cones or a car.
    * param x_coord: x position relative to parent frame
    * param y_coord: y position relative to parent frame
    * param ID: Unique for markers in the same frame
    * param header: passed in because creating time is dumb
    * return: Marker
    """

    marker = Marker()
    marker.header = header
    marker.ns = "current_path"
    marker.id = ID
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = x_coord
    marker.pose.position.y = y_coord
    marker.pose.position.z = z_coord
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # scale out of 1x1x1m
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.a = 1.0 # alpha
    # make the cone its own color and black if unknown
    if color == 0:
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    elif color == 1:
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 2 or color == 3:
        marker.color.r = 1.0
        marker.color.g = 0.7
        marker.color.b = 0.0
    else:
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0

    
    

    marker.lifetime = Duration(sec=1, nanosec=0)

    return marker

def cov_msg(
    x_coord: float, 
    y_coord: float, 
    z_coord: float,
    ID: int, 
    header: Header,
    x_scale: float,
    y_scale: float,
    z_scale: float,
    buffer: bool
) -> Marker: 
    """
    Creates a Marker object for cones or a car.
    * param x_coord: x position relative to parent frame
    * param y_coord: y position relative to parent frame
    * param ID: Unique for markers in the same frame
    * param header: passed in because creating time is dumb
    * return: Marker
    """

    marker = Marker()
    marker.header = header
    marker.ns = "current_path"
    marker.id = ID
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = x_coord
    marker.pose.position.y = y_coord
    marker.pose.position.z = z_coord
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # scale out of 1x1x1m
    marker.scale.x = x_scale
    marker.scale.y = y_scale
    marker.scale.z = z_scale

    marker.color.a = 0.3 # alpha
    if buffer:
        marker.color.r = 0.65
        marker.color.g = 0.65
        marker.color.b = 0.0
    else:
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

    marker.lifetime = Duration(sec=1, nanosec=0)

    return marker

def multivariate_multiply(m1, c1, m2, c2):
    """
    Taken from the filterpy library
    MIT Licence
    https://github.com/rlabbe/filterpy
    Multiplies the two multivariate Gaussians together and returns the
    results as the tuple (mean, covariance).
    Examples
    --------
    .. code-block:: Python
        m, c = multivariate_multiply([7.0, 2], [[1.0, 2.0], [2.0, 1.0]],
                                     [3.2, 0], [[8.0, 1.1], [1.1,8.0]])
    Parameters
    ----------
    m1 : array-like
        Mean of first Gaussian. Must be convertable to an 1D array via
        numpy.asarray(), For example 6, [6], [6, 5], np.array([3, 4, 5, 6])
        are all valid.
    c1 : matrix-like
        Covariance of first Gaussian. Must be convertable to an 2D array via
        numpy.asarray().
     m2 : array-like
        Mean of second Gaussian. Must be convertable to an 1D array via
        numpy.asarray(), For example 6, [6], [6, 5], np.array([3, 4, 5, 6])
        are all valid.
    c2 : matrix-like
        Covariance of second Gaussian. Must be convertable to an 2D array via
        numpy.asarray().
    Returns
    -------
    m : ndarray
        mean of the result
    c : ndarray
        covariance of the result
    """

    C1 = np.asarray(c1)
    C2 = np.asarray(c2)
    M1 = np.asarray(m1)
    M2 = np.asarray(m2)

    sum_inv = np.linalg.inv(C1+C2)
    C3 = np.dot(C1, sum_inv).dot(C2)

    M3 = (np.dot(C2, sum_inv).dot(M1) +
          np.dot(C1, sum_inv).dot(M2))

    return M3, C3