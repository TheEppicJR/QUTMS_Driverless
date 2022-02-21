# COPIED FROM ZED_CAMERA
# TODO: figure out a way to share python code among ROS packages
from dataclasses import dataclass
import numpy as np
from typing import Tuple
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from math import sqrt, sin, cos

@dataclass
class Point:
    x: float
    y: float

    def __add__(self, other: "Point") -> "Point":
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Point") -> "Point":
        return Point(self.x - other.x, self.y - other.y)

    def __truediv__(self, divisor: int) -> "Point":
        return Point(int(round(self.x/divisor)), int(round(self.y/divisor)))
    
    # NEW METHODS ADDED
    def __mul__(self, multiplier: int) -> "Point":
        return Point(self.x*multiplier, self.y*multiplier)
    
    def to_tuple(self) -> Tuple:
        return (self.x, self.y)


class PointWithCov:
    def __init__(self,
        loc_x: float,
        loc_y: float,
        loc_z: float,
        loc_cov: np.array,
        header: Header
        ) -> None:
        self.loc_x: float = loc_x
        self.loc_y: float = loc_y
        self.loc_z: float = loc_z
        self.loc_cov: np.array = loc_cov
        self.global_x: float = None
        self.global_y: float = None
        self.global_z: float = None
        self.global_cov: np.array = None
        self.header: Header = header
        self.nMeasurments: int = 0
    
    def translate(self, x, y, z, theta, g_cov):
        s, c = sin(theta), cos(theta)
        rotation_matrix = [[c, -1*s, 0],[s, c, 0], [0, 0, 1]]
        new_cov = rotation_matrix @ self.loc_cov @ rotation_matrix.T
        self.global_cov = new_cov + g_cov
        self.global_x = x + self.loc_x * c - self.loc_y * s
        self.global_y = y + self.loc_y * c + self.loc_x * s
        self.global_z = z + self.loc_z
        self.coords = (self.global_x, self.global_y)

    def update(self, other:"PointWithCov"):
        m3, c3 = multivariate_multiply([self.global_x, self.global_y, self.global_z], self.global_cov, [other.global_x, other.global_y, other.global_z], other.global_cov)
        self.global_cov = c3
        self.global_x = m3[0]
        self.global_y = m3[1]
        self.global_z = m3[2]
        self.coords = (self.global_x, self.global_y)
        self.nMeasurments += 1

    def dist(self, other:"PointWithCov"):
        return sqrt((self.global_x-other.global_x)**2+(self.global_y-other.global_y)**2+(self.global_z-other.global_z)**2)

    def getMarker(self, id: int):
        return point_msg(self.global_x, self.global_y, self.global_z, id, self.header)

    def getCov(self, id: int):
        # make a deformed sphere at 3 sigma of the variance in each axis (the diagnal elements of the covariance matrix are squared so we gotta sqrt)
        return cov_msg(self.global_x, self.global_y, self.global_z, id, self.header, 3*sqrt(self.global_cov[0,0]), 3*sqrt(self.global_cov[1,1]), 3*sqrt(self.global_cov[2,2]))

    def __len__(self):
        return len(self.coords)

    def __getitem__(self, i):
        return self.coords[i]

    def __repr__(self):
        return 'Item({}, {}, {})'.format(self.coords[0], self.coords[1], self.data)


def point_msg(
    x_coord: float, 
    y_coord: float, 
    z_coord: float,
    ID: int, 
    header: Header,
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
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0

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