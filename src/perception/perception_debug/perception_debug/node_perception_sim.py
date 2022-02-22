# import ROS2 libraries
from tkinter import N
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time, Duration
from cv_bridge import CvBridge
from rclpy.clock import ClockType
import message_filters
# import ROS2 message libraries
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry




# For odometry message
from transforms3d.euler import quat2euler


# Matrix/Array library
import numpy as np
# other python modules
import math
from random import gauss, random
from math import sin, cos, sqrt
from typing import List
import sys
import os
import getopt
import logging
import datetime
import pathlib
import threading
import time

LOGGER = logging.getLogger(__name__)

# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped, Waypoint, WaypointsArray, PointWithCovarianceStamped, PointWithCovarianceStampedArray
from fs_msgs.msg import ControlCommand, Track



def normalize_angle(angle: float) -> float:
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

class SimNode(Node):
    def __init__(self):
        super().__init__("cone_cov")

        # create the critical subscriptions
        self.create_subscription(Track, "/testing_only/track", self.mapCallback, 10)
        sub = message_filters.Subscriber(self, Odometry, "/testing_only/odom")
        self.cache = message_filters.Cache(sub, 100)

        self.lidar_publisher_cov: Publisher = self.create_publisher(PointWithCovarianceStampedArray, "/lidar/cone_detection_cov", 1)
        self.vision_publisher_cov: Publisher = self.create_publisher(PointWithCovarianceStampedArray, "/vision/cone_detection_cov", 1)
        self.lidar_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "/lidar/cone_detection", 1)
        self.vision_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "/vision/cone_detection", 1)

        self.visiondelay = 0.25
        self.visiondelayvar = 0.05
        self.visionfreq = 1/30
        self.lidardelay = 0.15
        self.lidardelayvar = 0.05
        self.lidarfreq = 1/10

        self.lidartime = time.time()
        self.visiontime = time.time()

        self.lidarcov = np.array([[1.19119729e+00, 8.68892102e-02, 7.36599069e-06],[8.68892102e-02, 1.93418457e-01, 1.20503303e-05],[7.36599069e-06, 1.20503303e-05, 1.64650607e-07]])

        self.visioncov = np.array([[ 3.07968324e-01,  2.49363466e-02, -1.28956374e-05], [ 2.49363466e-02, 1.59772299e-01, -9.15668618e-07], [-1.28956374e-05, -9.15668618e-07,  1.86769833e-07]])

        self.map = None

    def genRandomTime(self, delta, var):
        return delta - var + 2 * var * gauss(1, 0.3)

    def mapCallback(self, track_msg: Track):
        self.map = track_msg.track

    def publishdetections(self):
        if self.lidartime < time.time():
            self.lidartime = time.time() + self.genRandomTime(self.lidarfreq, self.lidardelayvar/3)
            oldodom = self.cache.getElemBeforeTime(self.get_clock().now() - Duration(nanoseconds=self.genRandomTime(self.lidardelay, self.lidardelayvar)*10**9))
            frontCones = self.getFrontConeObstacles(self.map, 30, oldodom)
            returnobj = PointWithCovarianceStampedArray()
            returnobjcone = ConeDetectionStamped()
            if oldodom is not None:
                returnobj.points, returnobjcone.cones = self.randomiseConePos(frontCones, self.lidarcov, oldodom)
                self.lidar_publisher_cov.publish(returnobj)
                returnobjcone.header = oldodom.header
                self.lidar_publisher.publish(returnobjcone)

        if self.visiontime < time.time():
            self.visiontime = time.time() + self.genRandomTime(self.visionfreq, self.visiondelayvar/3)
            oldodom = self.cache.getElemBeforeTime(self.get_clock().now() - Duration(nanoseconds=self.genRandomTime(self.visiondelay, self.visiondelayvar)*10**9))
            frontCones = self.getFrontConeObstacles(self.map, 30, oldodom)
            returnobj = PointWithCovarianceStampedArray()
            returnobjcone = ConeDetectionStamped()
            if oldodom is not None:
                returnobj.points, returnobjcone.cones = self.randomiseConePos(frontCones, self.visioncov, oldodom)
                self.vision_publisher_cov.publish(returnobj)
                returnobjcone.header = oldodom.header
                self.vision_publisher.publish(returnobjcone)

    def randomiseConePos(self, cones, cov: np.array, odom):
        pointswithcov: List[PointWithCovarianceStamped] = []
        conesout: List[Cone] = []
        orientation_q = odom.pose.pose.orientation
        orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        (roll, pitch, yaw)  = quat2euler(orientation_list)

        ns = sin(-yaw)
        nc = cos(-yaw)

        carPosX = odom.pose.pose.position.x
        carPosY = odom.pose.pose.position.y
        carPosZ = odom.pose.pose.position.y
        for cone in cones:
            # matrix multiply the covariance matrix with a random vector
            #noise = cov @ np.array([np.random.random()-0.5,np.random.random()-0.5,np.random.random()-0.5])
            #x, y, z = cone.location.x + noise[0], cone.location.y + noise[1], cone.location.z + noise[2]
            # the 0.3 is pretty arbitrary
            x, y, z = cone.location.x + sqrt(cov[0][0]) * gauss(0, 0.3), cone.location.y + sqrt(cov[1][1]) * gauss(0, 0.3), cone.location.z + sqrt(cov[2][2]) * gauss(0, 0.3)
            gax, gay, gaz = x - carPosX, y - carPosY, z
            lax, lay, laz = gax*nc-gay*ns, gay*nc+gax*ns, gaz
            thepoint = Point()
            thepoint.x = lax
            thepoint.y = lay
            thepoint.z = laz
            coneout = Cone()
            coneout.location = thepoint
            coneout.color = cone.color
            conesout.append(coneout)
            pwcs = PointWithCovarianceStamped()
            pwcs.header = odom.header
            pwcs.header.frame_id = "map"
            pwcs.position = thepoint
            pwcs.covariance = cov.flatten()
            pwcs.color = cone.color
            pointswithcov.append(pwcs)
        return pointswithcov, conesout


    def getFrontConeObstacles(self, map, frontDist, odom_msg):
        if not map or map is None or odom_msg is None:
            return []
        
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        (roll, pitch, yaw)  = quat2euler(orientation_list)

        carPosX = odom_msg.pose.pose.position.x
        carPosY = odom_msg.pose.pose.position.y

        headingVector = self.getHeadingVector(yaw)

        headingVectorOrt = [-headingVector[1], headingVector[0]]

        behindDist = 0.5
        carPosBehindPoint = [carPosX - behindDist * headingVector[0], carPosY - behindDist * headingVector[1]]


        frontDistSq = frontDist ** 2

        frontConeList = []
        for cone in map:
            if (headingVectorOrt[0] * (cone.location.y - carPosBehindPoint[1]) - headingVectorOrt[1] * (cone.location.x - carPosBehindPoint[0])) < 0:
                # make it a little inconsistant at farther ranges
                if ((cone.location.x - carPosX) ** 2 + (cone.location.y - carPosY) ** 2) < frontDistSq and sqrt((cone.location.x - carPosX) ** 2 + (cone.location.y - carPosY) ** 2)*gauss(1, 0.3) < 15:
                    frontConeList.append(cone)
        return frontConeList

    def getHeadingVector(self, carPosYaw):
        headingVector = [1.0, 0]
        carRotMat = np.array([[math.cos(carPosYaw), -math.sin(carPosYaw)], [math.sin(carPosYaw), math.cos(carPosYaw)]])
        headingVector = np.dot(carRotMat, headingVector)
        return headingVector



def main(args=sys.argv[1:]):
    # defaults args
    loglevel = 'info'
    print_logs = False

    numeric_level = getattr(logging, loglevel.upper(), None)

    # setting up logging
    path = str(pathlib.Path(__file__).parent.resolve())
    if not os.path.isdir(path + '/logs'):
        os.mkdir(path + '/logs')

    date = datetime.datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
    logging.basicConfig(
        filename=f'{path}/logs/{date}.log',
        filemode='w',
        format='%(asctime)s | %(levelname)s:%(name)s: %(message)s',
        datefmt='%I:%M:%S %p',
        # encoding='utf-8',
        level=numeric_level,
    )

    # terminal stream
    if print_logs:
        stdout_handler = logging.StreamHandler(sys.stdout)
        LOGGER.addHandler(stdout_handler)

    LOGGER.info(f'args = {args}')
    
    # begin ros node
    rclpy.init(args=args)

    node = SimNode()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(100)

    try:
        while rclpy.ok():
            node.publishdetections()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()

    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main(sys.argv[1:])