# import ROS2 libraries
import imp
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
import rclpy.logging
from cv_bridge import CvBridge
import message_filters
# import ROS2 message libraries
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration, Time
from nav_msgs.msg import Odometry
# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped
from fs_msgs.msg import ControlCommand

# other python modules
from math import sqrt, atan2, pi, sin, cos, atan
import cv2
import numpy as np
import matplotlib.pyplot as plt # plotting splines
import scipy.interpolate as scipy_interpolate # for spline calcs
from typing import Tuple, List, Optional
import time
import sys
import os
import getopt
import logging
import datetime
import pathlib

from transforms3d.euler import quat2euler

# import required sub modules
from .point import Point

# initialise logger
LOGGER = logging.getLogger(__name__)

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()

# image display geometry
SCALE = 20
WIDTH = 20*SCALE # 10m either side
HEIGHT = 20*SCALE # 20m forward
ORIGIN = Point(0, 0)
IMG_ORIGIN = Point(int(WIDTH/2), HEIGHT)

# display colour constants
Colour = Tuple[int, int, int]
YELLOW_DISP_COLOUR: Colour = (0, 255, 255) # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0) # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 165, 255) # bgr - orange

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW



def robot_pt_to_img_pt(x: float, y: float) -> Point:
    """
    Converts a relative depth from the camera into image coords
    * param x: x coord
    * param y: y coord
    * return: Point int pixel coords
    """
    return Point(
        int(round(WIDTH/2 - y*SCALE)),
        int(round(HEIGHT - x*SCALE)),
    )


def dist(a: Point, b: Point) -> float:
    return sqrt((a.x-b.x)**2 + (a.y-b.y)**2)


def cone_to_point(cone: Cone) -> Point:
    return Point(
        cone.location.x,
        cone.location.y,
    )


def midpoint(p1: list, p2: list):
    """
    Retrieve midpoint between two points 
    * param p1: [x,y] coords of point 1
    * param p2: [x,y] coords of point 2
    * return: x,y tuple of midpoint coord
    """
    return (p1[0]+p2[0])/2, (p1[1]+p2[1])/2


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

        self.logger = self.get_logger()

        # subscribers
        cones_sub = message_filters.Subscriber(
            self, ConeDetectionStamped, "/detector/cone_detection"
        )
        lidar_cones_sub = message_filters.Subscriber(
            self, ConeDetectionStamped, "/cone_sensing/cones"
        )
        

        # synchronizer = message_filters.TimeSynchronizer(
        #     fs=[cones_sub, lidar_cones_sub],
        #     queue_size=100,
        # )
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            fs=[cones_sub, lidar_cones_sub],
            queue_size=60,
            slop=0.4
        )
        synchronizer.registerCallback(self.callback)

        # publishers
        self.debug_img_publisher: Publisher = self.create_publisher(Image, "/cone_pipeline/debug_img", 1)
        #self.path_publisher: Publisher = self.create_publisher(MarkerArray, "/cone_pipeline/target_array", 1)

        LOGGER.info("---Cone Pipeline Node Initalised---")
        self.logger.debug("---Cone Pipeline Node Initalised---")


    def callback(self, cone_msg: ConeDetectionStamped, lidar_cone_msg: ConeDetectionStamped):
        LOGGER.info("Received detection")
        self.logger.debug("Received detection")

        cones: List[Cone] = cone_msg.cones
        lidar_cones: List[Cone] = lidar_cone_msg.cones
        # create black image
        debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)


        for cone in cones:
            if cone.color == Cone.YELLOW:
                colour = YELLOW_DISP_COLOUR
            elif cone.color == Cone.BLUE:
                colour = BLUE_DISP_COLOUR
            else:
                colour = (255, 255, 255)
            
            # draws location of cone w/ colour
            cv2.drawMarker(
                debug_img, 
                robot_pt_to_img_pt(cone.location.x + 1.2, cone.location.y).to_tuple(),
                colour,
                markerType=cv2.MARKER_SQUARE,
                markerSize=3,
                thickness=3
            )

        for cone in lidar_cones:
            colour = ORANGE_DISP_COLOUR
            cv2.drawMarker(
                debug_img, 
                robot_pt_to_img_pt(cone.location.x + 1.2, cone.location.y).to_tuple(),
                colour,
                markerType=cv2.MARKER_SQUARE,
                markerSize=3,
                thickness=3
            )

        text_vel = f"Lidar N: {len(lidar_cones)}"
        cv2.putText(
            debug_img, text_vel, (10, HEIGHT-10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
        )
        # timedelta = lidar_cone_msg.header.stamp-cone_msg.header.stamp
        # text_time = f"dT: {timedelta}"
        # cv2.putText(
        #     debug_img, text_time, (10, HEIGHT-25),
        #     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
        # )
        
        self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))



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

    node = ConePipeline()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])

