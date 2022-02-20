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
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from sklearn import cluster
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry
# import custom message libraries
from driverless_msgs.msg import Cone as QUTCone
from driverless_msgs.msg import ConeDetectionStamped
from fs_msgs.msg import ControlCommand, Track

# other python modules
from math import sqrt, atan2, pi, sin, cos, atan
import cv2
import numpy as np
import matplotlib.pyplot as plt # plotting splines
import scipy.interpolate as scipy_interpolate # for spline calcs
from typing import Tuple, List, Optional
import sys
import os
import getopt
import logging
import pathlib
import time

from transforms3d.euler import quat2euler

# import required sub modules
from .point import Point
from . import kmeans_clustering as km

# initialise logger
LOGGER = logging.getLogger(__name__)

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()


LEFT_CONE_COLOUR = QUTCone.BLUE
RIGHT_CONE_COLOUR = QUTCone.YELLOW


def marker_msg(
    x_coord: float, 
    y_coord: float, 
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
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # scale out of 1x1x1m
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.a = 1.0 # alpha
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    marker.lifetime = Duration(sec=1, nanosec=0)

    return marker


class ConePipeline(Node):
    def __init__(self):
        super().__init__("cone_pipeline")

        self.tx: List[float] = []
        self.ty: List[float] = []
        self.yellow_x: List[float] = []
        self.yellow_y: List[float] = []
        self.blue_x: List[float] = []
        self.blue_y: List[float] = []
        self.yx: List[float] = []
        self.yy: List[float] = []
        self.bx: List[float] = []
        self.by: List[float] = []
        self.path_markers: List[Marker] = []

        self.spline_len: int = 4000

        self.fig = plt.figure()

        self.logger = self.get_logger()

        self.create_subscription(Track, "/testing_only/track", self.map_callback, 10)

        # subscribers
        cones_sub = message_filters.Subscriber(
            self, ConeDetectionStamped, "/detector/cone_detection"
        )
        self.actualcone = message_filters.Cache(cones_sub, 100)

        lidar_cones_sub = message_filters.Subscriber(
            self, ConeDetectionStamped, "/lidar/cone_detection"
        )
        lidar_cones_sub.registerCallback(self.callback)

        odom_sub = message_filters.Subscriber(self, Odometry, "/testing_only/odom")
        self.actualodom = message_filters.Cache(odom_sub, 100)

        self.plot_img_publisher: Publisher = self.create_publisher(Image, "/cone_pipeline/plot_img", 1)

        self.cones = []
        self.num_cones = 1
        self.max_range = 18


        # publishers
        self.debug_img_publisher: Publisher = self.create_publisher(Image, "/cone_pipeline/debug_img", 1)
        #self.path_publisher: Publisher = self.create_publisher(MarkerArray, "/cone_pipeline/target_array", 1)

        LOGGER.info("---Cone Pipeline Node Initalised---")
        self.logger.debug("---Cone Pipeline Node Initalised---")




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

