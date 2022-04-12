# COPIED FROM ZED_CAMERA
# TODO: figure out a way to share python code among ROS packages
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point as PointMsg
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry
from math import sqrt, sin, cos
from transforms3d.euler import quat2mat
from typing import Iterable, Optional, Tuple

from geometry_msgs.msg import (Pose, PoseStamped,
                               PoseWithCovarianceStamped, TransformStamped,
                               Vector3Stamped)
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

    def xydist(self, x, y):
        return sqrt((self.loc_x-x)**2 + (self.loc_y-y)**2)

    def dist(self, other:"PointWithCov"):
        return sqrt((self.global_x-other.global_x)**2+(self.global_y-other.global_y)**2+(self.global_z-other.global_z)**2)

    # should add cone color to this
    def getMarker(self, id: int, z_offset: float, header: Header):
        return point_msg(self.global_x, self.global_y, self.global_z-z_offset, id, header, self.color)

    def getCov(self, id: int, buffer: bool, z_offset: float, header: Header):
        # make a deformed sphere at 3 sigma of the variance in each axis (the diagnal elements of the covariance matrix are squared so we gotta sqrt)
        return cov_msg(self.global_x, self.global_y, self.global_z-z_offset, id, header, 3*sqrt(abs(self.global_cov[0,0])), 3*sqrt(abs(self.global_cov[1,1])), 3*sqrt(abs(self.global_cov[2,2])), buffer)

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
    def __init__(self, p1: PointWithCov, p2: PointWithCov, p3: PointWithCov) -> None:
        self.p1: PointWithCov = p1
        self.p2: PointWithCov = p2
        self.p3: PointWithCov = p3
        self.calcCentroid()

    def calcCentroid(self):
        self.x = (self.p1.global_x + self.p2.global_x + self.p3.global_x) / 3
        self.y = (self.p1.global_y + self.p2.global_y + self.p3.global_y) / 3
        self.z = (self.p1.global_z + self.p2.global_z + self.p3.global_z) / 3

    def getEdges(self):
        return (Edge(self.p1, self.p2), Edge(self.p2, self.p3), Edge(self.p3, self.p1))

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
    # marker.header.frame_id = "map"
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

# Copyright 2008 Willow Garage, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# author: Wim Meeussen




def to_msg_msg(msg):
    return msg

def from_msg_msg(msg):
    return msg

def transform_covariance(cov_in, transform):
    """
    Apply a given transform to a covariance matrix.
    :param cov_in: Covariance matrix
    :param transform: The transform that will be applies
    :returns: The transformed covariance matrix
    """
    # Converting the Quaternion to a Rotation Matrix first
    # Taken from: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    q0 = transform.transform.rotation.w
    q1 = transform.transform.rotation.x
    q2 = transform.transform.rotation.y
    q3 = transform.transform.rotation.z

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # Code reference: https://github.com/ros2/geometry2/pull/430
    # Mathematical Reference:
    # A. L. Garcia, “Linear Transformations of Random Vectors,” in Probability,
    # Statistics, and Random Processes For Electrical Engineering, 3rd ed.,
    # Pearson Prentice Hall, 2008, pp. 320–322.

    R = np.array([[r00, r01, r02],
                  [r10, r11, r12],
                  [r20, r21, r22]])

    R_transpose = np.transpose(R)

    cov_11 = np.array([cov_in[:3], cov_in[6:9], cov_in[12:15]])
    cov_12 = np.array([cov_in[3:6], cov_in[9:12], cov_in[15:18]])
    cov_21 = np.array([cov_in[18:21], cov_in[24:27], cov_in[30:33]])
    cov_22 = np.array([cov_in[21:24], cov_in[27:30], cov_in[33:]])

    # And we perform the transform
    result_11 = R @ cov_11 @ R_transpose
    result_12 = R @ cov_12 @ R_transpose
    result_21 = R @ cov_21 @ R_transpose
    result_22 = R @ cov_22 @ R_transpose

    cov_out = PoseWithCovarianceStamped()

    cov_out.pose.covariance[0] = result_11[0][0]
    cov_out.pose.covariance[1] = result_11[0][1]
    cov_out.pose.covariance[2] = result_11[0][2]
    cov_out.pose.covariance[6] = result_11[1][0]
    cov_out.pose.covariance[7] = result_11[1][1]
    cov_out.pose.covariance[8] = result_11[1][2]
    cov_out.pose.covariance[12] = result_11[2][0]
    cov_out.pose.covariance[13] = result_11[2][1]
    cov_out.pose.covariance[14] = result_11[2][2]

    cov_out.pose.covariance[3] = result_12[0][0]
    cov_out.pose.covariance[4] = result_12[0][1]
    cov_out.pose.covariance[5] = result_12[0][2]
    cov_out.pose.covariance[9] = result_12[1][0]
    cov_out.pose.covariance[10] = result_12[1][1]
    cov_out.pose.covariance[11] = result_12[1][2]
    cov_out.pose.covariance[15] = result_12[2][0]
    cov_out.pose.covariance[16] = result_12[2][1]
    cov_out.pose.covariance[17] = result_12[2][2]

    cov_out.pose.covariance[18] = result_21[0][0]
    cov_out.pose.covariance[19] = result_21[0][1]
    cov_out.pose.covariance[20] = result_21[0][2]
    cov_out.pose.covariance[24] = result_21[1][0]
    cov_out.pose.covariance[25] = result_21[1][1]
    cov_out.pose.covariance[26] = result_21[1][2]
    cov_out.pose.covariance[30] = result_21[2][0]
    cov_out.pose.covariance[31] = result_21[2][1]
    cov_out.pose.covariance[32] = result_21[2][2]

    cov_out.pose.covariance[21] = result_22[0][0]
    cov_out.pose.covariance[22] = result_22[0][1]
    cov_out.pose.covariance[23] = result_22[0][2]
    cov_out.pose.covariance[27] = result_22[1][0]
    cov_out.pose.covariance[28] = result_22[1][1]
    cov_out.pose.covariance[29] = result_22[1][2]
    cov_out.pose.covariance[33] = result_22[2][0]
    cov_out.pose.covariance[34] = result_22[2][1]
    cov_out.pose.covariance[35] = result_22[2][2]

    return cov_out.pose.covariance


def _build_affine(
        rotation: Optional[Iterable] = None,
        translation: Optional[Iterable] = None) -> np.ndarray:
    """
    Build an affine matrix from a quaternion and a translation.
    :param rotation: The quaternion as [w, x, y, z]
    :param translation: The translation as [x, y, z]
    :returns: The quaternion and the translation array
    """
    affine = np.eye(4)
    if rotation is not None:
        affine[:3, :3] = _get_mat_from_quat(np.asarray(rotation))
    if translation is not None:
        affine[:3, 3] = np.asarray(translation)
    return affine


def _transform_to_affine(transform: TransformStamped) -> np.ndarray:
    """
    Convert a `TransformStamped` to a affine matrix.
    :param transform: The transform that should be converted
    :returns: The affine transform
    """
    transform = transform.transform
    transform_rotation_matrix = [
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z
    ]
    transform_translation = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z
    ]
    return _build_affine(transform_rotation_matrix, transform_translation)


def _get_mat_from_quat(quaternion: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to a rotation matrix.
    This method is based on quat2mat from https://github.com
    f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/quaternions.py#L101 ,
    since that library is not available via rosdep.
    :param quaternion: A numpy array containing the w, x, y, and z components of the quaternion
    :returns: The rotation matrix
    """
    Nq = np.sum(np.square(quaternion))
    if Nq < np.finfo(np.float64).eps:
        return np.eye(3)

    XYZ = quaternion[1:] * 2.0 / Nq
    wXYZ = XYZ * quaternion[0]
    xXYZ = XYZ * quaternion[1]
    yYZ = XYZ[1:] * quaternion[2]
    zZ = XYZ[2] * quaternion[3]

    return np.array(
        [[1.0-(yYZ[0]+zZ), xXYZ[1]-wXYZ[2], xXYZ[2]+wXYZ[1]],
         [xXYZ[1]+wXYZ[2], 1.0-(xXYZ[0]+zZ), yYZ[1]-wXYZ[0]],
         [xXYZ[2]-wXYZ[1], yYZ[1]+wXYZ[0], 1.0-(xXYZ[0]+yYZ[0])]])


def _get_quat_from_mat(rot_mat: np.ndarray) -> np.ndarray:
    """
    Convert a rotation matrix to a quaternion.
    This method is a copy of mat2quat from https://github.com
    f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/quaternions.py#L150 ,
    since that library is not available via rosdep.
    Method from
    Bar-Itzhack, Itzhack Y. (2000), "New method for extracting the
    quaternion from a rotation matrix", AIAA Journal of Guidance,
    Control and Dynamics 23(6):1085-1087 (Engineering Note), ISSN
    0731-5090
    :param rot_mat: A roatation matrix
    :returns: An quaternion
    """
    # Decompose rotation matrix
    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = rot_mat.flat
    # Create matrix
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
    ) / 3.0
    vals, vecs = np.linalg.eigh(K)
    # Select largest eigenvector and reorder to w,x,y,z
    q = vecs[[3, 0, 1, 2], np.argmax(vals)]
    # Invert quaternion if w is negative (results in positive w)
    if q[0] < 0:
        q *= -1
    return q


def _decompose_affine(affine: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Decompose an affine transformation into a quaternion and the translation.
    :param affine: The affine transformation matrix
    :returns: The quaternion and the translation array
    """
    return _get_quat_from_mat(affine[:3, :3]), affine[:3, 3]


# PointStamped
def do_transform_point(
        point: PointMsg,
        transform: TransformStamped) -> PointMsg:
    """
    Transform a `PointStamped` using a given `TransformStamped`.
    :param point: The point
    :param transform: The transform
    :returns: The transformed point
    """
    _, point = _decompose_affine(
        np.matmul(
            _transform_to_affine(transform),
            _build_affine(translation=[
                point.x,
                point.y,
                point.z
            ])))

    res = PointMsg()
    res.x = point[0]
    res.y = point[1]
    res.z = point[2]
    return res



# Vector3Stamped
def do_transform_vector3(
        vector3: Vector3Stamped,
        transform: TransformStamped) -> Vector3Stamped:
    """
    Transform a `Vector3Stamped` using a given `TransformStamped`.
    :param vector3: The vector3
    :param transform: The transform
    :returns: The transformed vector3
    """
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    _, point = _decompose_affine(
        np.matmul(
            _transform_to_affine(transform),
            _build_affine(translation=[
                vector3.vector.x,
                vector3.vector.y,
                vector3.vector.z
            ])))
    res = Vector3Stamped()
    res.vector.x = point[0]
    res.vector.y = point[1]
    res.vector.z = point[2]
    res.header = transform.header
    return res




# Pose
def do_transform_pose(
        pose: Pose,
        transform: TransformStamped) -> Pose:
    """
    Transform a `Pose` using a given `TransformStamped`.
    This method is used to share the tranformation done in
    `do_transform_pose_stamped()` and `do_transform_pose_with_covariance_stamped()`
    :param pose: The pose
    :param transform: The transform
    :returns: The transformed pose
    """
    quaternion, point = _decompose_affine(
        np.matmul(
            _transform_to_affine(transform),
            _build_affine(
                translation=[
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                ],
                rotation=[
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z])))
    res = Pose()
    res.position.x = point[0]
    res.position.y = point[1]
    res.position.z = point[2]
    res.orientation.w = quaternion[0]
    res.orientation.x = quaternion[1]
    res.orientation.y = quaternion[2]
    res.orientation.z = quaternion[3]
    return res


# PoseStamped
def do_transform_pose_stamped(
        pose: PoseStamped,
        transform: TransformStamped) -> PoseStamped:
    """
    Transform a `PoseStamped` using a given `TransformStamped`.
    :param pose: The stamped pose
    :param transform: The transform
    :returns: The transformed pose stamped
    """
    res = PoseStamped()
    res.pose = do_transform_pose(pose.pose, transform)
    res.header = transform.header
    return res



# PoseWithCovarianceStamped
def do_transform_pose_with_covariance_stamped(
        pose: PoseWithCovarianceStamped,
        transform: TransformStamped) -> PoseWithCovarianceStamped:
    """
    Transform a `PoseWithCovarianceStamped` using a given `TransformStamped`.
    :param pose: The pose with covariance stamped
    :param transform: The transform
    :returns: The transformed pose with covariance stamped
    """
    res = PoseWithCovarianceStamped()
    res.pose.pose = do_transform_pose(pose.pose.pose, transform)
    res.pose.covariance = transform_covariance(pose.pose.covariance, transform)
    res.header = transform.header
    return res

