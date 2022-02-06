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
from fs_msgs.msg import ControlCommand

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

from transforms3d.euler import quat2euler

# import required sub modules
from .point import Point
from . import kmeans_clustering as km

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
PURP_DISP_COLOUR: Colour = (230, 230, 250) # bgr - orange

LEFT_CONE_COLOUR = QUTCone.BLUE
RIGHT_CONE_COLOUR = QUTCone.YELLOW



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


def cone_to_point(cone: QUTCone) -> Point:
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
        self.actualcone = message_filters.Cache(cones_sub, 100)
        lidar_cones_sub = message_filters.Subscriber(
            self, ConeDetectionStamped, "/lidar/cone_detection"
        )
        lidar_cones_sub.registerCallback(self.callback)
        odom_sub = message_filters.Subscriber(self, Odometry, "/testing_only/odom")
        self.actualodom = message_filters.Cache(odom_sub, 100)

        self.cones = []
        self.num_cones = 1

        

        # publishers
        self.debug_img_publisher: Publisher = self.create_publisher(Image, "/cone_pipeline/debug_img", 1)
        #self.path_publisher: Publisher = self.create_publisher(MarkerArray, "/cone_pipeline/target_array", 1)

        LOGGER.info("---Cone Pipeline Node Initalised---")
        self.logger.debug("---Cone Pipeline Node Initalised---")


    def update_kmeans(self, new_cones, odom_msg: Odometry):
        self.odom_header = odom_msg.header

        w = odom_msg.pose.pose.orientation.w
        i = odom_msg.pose.pose.orientation.x
        j = odom_msg.pose.pose.orientation.y
        k = odom_msg.pose.pose.orientation.z

        # i, j, k angles in rad
        ai, aj, ak = quat2euler([w, i, j, k])

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        for cone in new_cones:
            x_dist = cone.location.x
            y_dist = cone.location.x
            a = cos(ak)
            b = sin(ak)
            c = y_dist + y*a - x*b
            d = x_dist + x*a + y*b
            nx = (d*a-c)/(a*a+b)

            ref_cone = QUTCone()
            # reference frame displacement with rotation 
            # uses k angle (z axis)
            ref_cone.location.x = nx
            ref_cone.location.y = (d - nx*a)/b
            ref_cone.location.z = 0.0
            ref_cone.color = 3
            self.cones.append(ref_cone)
        raw_x, raw_y = [], []
        for cone in self.cones:
            raw_x.append(cone.location.x)
            raw_y.append(cone.location.y)
        
        kms = km.kmeans_clustering(raw_x, raw_y, self.num_cones)
        self.clusters = kms.plot_cluster()

    def get_km_cones(self, odom_msg: Odometry):
        
        # header used to create markers
        self.odom_header = odom_msg.header

        w = odom_msg.pose.pose.orientation.w
        i = odom_msg.pose.pose.orientation.x
        j = odom_msg.pose.pose.orientation.y
        k = odom_msg.pose.pose.orientation.z

        # i, j, k angles in rad
        ai, aj, ak = quat2euler([w, i, j, k])

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        ref_cones: List[QUTCone] = []
        for cone in self.clusters:
            # displacement from car to cone
            x_dist = cone[0][0] - x
            y_dist = cone[0][1] - y

            ref_cone = QUTCone()
            # reference frame displacement with rotation 
            # uses k angle (z axis)
            ref_cone.location.x = x_dist*cos(ak) + y_dist*sin(ak)
            ref_cone.location.y = y_dist*cos(ak) - x_dist*sin(ak)
            ref_cone.location.z = 0.0
            ref_cone.color = 3

            if ref_cone.location.x > 0 and ref_cone.location.x < self.max_range \
                and ref_cone.location.y > -9 and ref_cone.location.y < 9:
                ref_cones.append(ref_cone)
        return ref_cones


    def callback(self, lidar_cone_msg: ConeDetectionStamped):
        LOGGER.info("Received detection")
        self.logger.debug("Received detection")


        lidar_scan_time: Time = Time.from_msg(lidar_cone_msg.header.stamp)

        a: ConeDetectionStamped = self.actualcone.getElemAfterTime(lidar_scan_time)
        o: Odometry = self.actualodom.getElemAfterTime(lidar_scan_time)
        self.num_cones = len(a.cones)
        print("it does something")
        #b = self.actualcone.getElemBeforeTime(lidar_scan_time)
        try:
            cones: List[QUTCone] = a.cones
            lidar_cones: List[QUTCone] = lidar_cone_msg.cones
            # create black image
            debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

            self.update_kmeans(lidar_cones, o)

            for cone in cones:
                if cone.color == QUTCone.YELLOW:
                    colour = YELLOW_DISP_COLOUR
                elif cone.color == QUTCone.BLUE:
                    colour = BLUE_DISP_COLOUR
                else:
                    colour = (255, 255, 255)

                # draws location of cone w/ colour
                cv2.drawMarker(
                    debug_img, 
                    robot_pt_to_img_pt((cone.location.x - 1.2), cone.location.y).to_tuple(),
                    colour,
                    markerType=cv2.MARKER_SQUARE,
                    markerSize=3,
                    thickness=3
                )

            for cone in self.get_km_cones(o):
                colour = PURP_DISP_COLOUR
                cv2.drawMarker(
                    debug_img, 
                    robot_pt_to_img_pt(cone.location.x, cone.location.y).to_tuple(),
                    colour,
                    markerType=cv2.MARKER_SQUARE,
                    markerSize=3,
                    thickness=3
                )

            for cone in lidar_cones:
                colour = ORANGE_DISP_COLOUR
                cv2.drawMarker(
                    debug_img, 
                    robot_pt_to_img_pt(cone.location.x, cone.location.y).to_tuple(),
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
        except Exception as e: print(e)



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

