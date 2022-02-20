# import ROS2 libraries
import imp
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time
import rclpy.logging
from cv_bridge import CvBridge
import message_filters
# import ROS2 message libraries
from sklearn import cluster
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
# import custom message libraries
from driverless_msgs.msg import Cone as QUTCone
from driverless_msgs.msg import ConeDetectionStamped, PointWithCovarianceStamped, PointWithCovarianceStampedArray

from typing import List


# other python modules
from math import sqrt, atan2, pi, sin, cos, atan
import cv2
import numpy as np
import sys
import os
import getopt
import logging
import pathlib
import time

from transforms3d.euler import quat2euler

# import required sub modules
from .point import Point, PointWithCov
from . import kmeans_clustering as km
from . import kdtree

# initialise logger
LOGGER = logging.getLogger(__name__)

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()


LEFT_CONE_COLOUR = QUTCone.BLUE
RIGHT_CONE_COLOUR = QUTCone.YELLOW

class ConePipeline(Node):
    def __init__(self):
        super().__init__("cone_pipeline")

        self.logger = self.get_logger()

        self.create_subscription(PointWithCovarianceStampedArray, "/lidar/cone_detection_cov", self.lidarCallback, 10)
        self.create_subscription(PointWithCovarianceStampedArray, "/vision/cone_detection_cov", self.visionCallback, 10)

        self.filtered_cones_pub: Publisher = self.create_publisher(PointWithCovarianceStampedArray, "/cone_pipe/cone_detection_cov", 1)

        self.lidar_markers: Publisher = self.create_publisher(MarkerArray, "/cone_pipe/lidar_marker", 1)
        self.vision_markers: Publisher = self.create_publisher(MarkerArray, "/cone_pipe/vision_marker", 1)
        self.filtered_markers: Publisher = self.create_publisher(MarkerArray, "/cone_pipe/filtered_marker", 1)

        odom_sub = message_filters.Subscriber(self, Odometry, "/testing_only/odom")
        self.actualodom = message_filters.Cache(odom_sub, 100)

        self.printmarkers: bool = True

        self.conesKDTree: KDNode = None

        self.bufferKDTree: KDNode = None

        LOGGER.info("---Cone Pipeline Node Initalised---")
        self.logger.debug("---Cone Pipeline Node Initalised---")

    def getNearestOdom(self, stamp):
        # need to switch this over to a position from a EKF with covariance
        locodom = self.cache.getElemBeforeTime(stamp)
        cov = np.identity(3) * 0.0025 # standin covariance for ekf assuming the variance is sigma = 5cm with no covariance
        return locodom, cov

    def fuseCone(self, point):
        closestcone = self.conesKDTree.search_knn(point, 1)
        if closestcone[0][1] < 0.5:
            closestcone[0][0].update(point)
            self.conesKDTree.rebalance()
        else:
            self.bufferCone(point)
            

    def bufferCone(self, point):
        if self.bufferKDTree is not None:
            closestcone = self.bufferKDTree.search_knn(point, 1)
            if closestcone[0][1] < 0.5:
                pointnew = closestcone[0][0]
                pointnew.update(point)
                self.bufferKDTree.remove(pointnew)
                if self.conesKDTree is not None:
                    self.conesKDTree.add(pointnew)
                    self.conesKDTree.rebalance()
                else:
                    self.conesKDTree = kdtree.create([point])
                self.bufferKDTree.rebalance()
            else:
                self.bufferKDTree.add(point)
                self.bufferKDTree.rebalance()
        else:
            self.bufferKDTree = kdtree.create([point])

    def fusePoints(self, points):
        if self.conesKDTree is not None:
            for point in points:
                self.fuseCone(point)
        else:
            for point in points:
                self.bufferCone(point)
        
        
    def lidarCallback(self, points: PointWithCovarianceStampedArray):
        if len(points.points) > 0:
            msgs = self.printmarkers and self.lidar_markers.get_subscription_count() > 0
            header = points.points[0].header
            odomloc, odomcov = self.getNearestOdom(header.stamp)
            orientation_q = odomloc.pose.pose.orientation
            orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
            (roll, pitch, theta)  = quat2euler(orientation_list)
            x = odomloc.pose.pose.position.x
            y = odomloc.pose.pose.position.y
            z = odomloc.pose.pose.position.z
            
            conelist: List[PointWithCov] = []
            markers: List[Marker] = []
            for point in points.points:
                p = PointWithCov(point.position.x, point.position.y, point.position.z, np.array(point.covariance).reshape((3,3)), point.header)
                p.translate(x, y, z, theta, odomcov)
                conelist.append(p)
                if msgs:
                    markers.append(p.getCov)
                    markers.append(p.getMarker)
            if msgs:
                mkr = MarkerArray()
                mkr.markers = markers
                self.lidar_markers.publish(mkr)
            self.fusePoints(conelist)

    def visionCallback(self, points: PointWithCovarianceStampedArray):
        if len(points.points) > 0:
            msgs = self.printmarkers and self.vision_markers.get_subscription_count() > 0
            header = points.points[0].header
            odomloc, odomcov = self.getNearestOdom(header.stamp)
            orientation_q = odomloc.pose.pose.orientation
            orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
            (roll, pitch, theta)  = quat2euler(orientation_list)
            x = odomloc.pose.pose.position.x
            y = odomloc.pose.pose.position.y
            z = odomloc.pose.pose.position.z
            
            conelist: List[PointWithCov] = []
            markers: List[Marker] = []
            for point in points.points:
                p = PointWithCov(point.position.x, point.position.y, point.position.z, np.array(point.covariance).reshape((3,3)), point.header)
                p.translate(x, y, z, theta, odomcov)
                conelist.append(p)
                if msgs:
                    markers.append(p.getCov)
                    markers.append(p.getMarker)
            if msgs:
                mkr = MarkerArray()
                mkr.markers = markers
                self.vision_markers.publish(mkr)
            self.fusePoints(conelist)



def main(args=sys.argv[1:]):
    # defaults args
    loglevel = 'info'
    print_logs = False

    # processing args
    opts, arg = getopt.getopt(args, str(), ['log=', 'print_logs', 'length='])

    # TODO: provide documentation for different options
    for opt, arg in opts:
        if opt == '--log':
            loglevel = arg
        elif opt == '--print_logs':
            print_logs = True

    # validating args
    numeric_level = getattr(logging, loglevel.upper(), None)

    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % loglevel)

    # setting up logging
    path = str(pathlib.Path(__file__).parent.resolve())
    if not os.path.isdir(path + '/logs'):
        os.mkdir(path + '/logs')

    date = "hi"
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

    node = ConePipeline()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])

