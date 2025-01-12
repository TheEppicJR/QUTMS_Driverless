# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# import ROS2 message libraries
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
# import custom message libraries
from fs_msgs.msg import Track, Cone
from driverless_msgs.msg import SplinePoint, SplineStamped

# other python modules
from math import atan2, pi
import cv2
import numpy as np
import scipy.interpolate as scipy_interpolate # for spline calcs
from typing import Tuple, List
import time
import sys
import os
import getopt
import logging
import datetime
import pathlib

# for velocity display colour
from colour import Color

# initialise logger
LOGGER = logging.getLogger(__name__)

# for colour gradient based on intensity
MAX_ANGLE = 0.15
red = Color("red")
blue = Color("blue")
col_range = list(blue.range_to(red, 100))


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def approximate_b_spline_path(
    x: list, 
    y: list, 
    n_path_points: int,
    degree: int = 3
) -> Tuple[list, list]:
    """
    ADAPTED FROM: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/BSplinePath/bspline_path.py \n
    Approximate points with a B-Spline path
    * param x: x position list of approximated points
    * param y: y position list of approximated points
    * param n_path_points: number of path points
    * param degree: (Optional) B Spline curve degree
    * return: x and y position list of the result path
    """

    t: int = range(len(x))
    # interpolate for the length of the input cone list
    x_list = list(scipy_interpolate.splrep(t, x, k=degree))
    y_list = list(scipy_interpolate.splrep(t, y, k=degree))

    # add 4 'zero' components to align matrices
    x_list[1] = x + [0.0, 0.0, 0.0, 0.0]
    y_list[1] = y + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, n_path_points)
    spline_x = scipy_interpolate.splev(ipl_t, x_list)
    spline_y = scipy_interpolate.splev(ipl_t, y_list)

    return spline_x, spline_y


def midpoint(
    p1: List[float], 
    p2: List[float]
) -> Tuple[float]:
    """
    Retrieve midpoint between two points 
    * param p1: [x,y] coords of point 1
    * param p2: [x,y] coords of point 2
    * return: x,y tuple of midpoint coord
    """
    return (p1[0]+p2[0])/2, (p1[1]+p2[1])/2


def angle(
    p1: List[float], 
    p2: List[float]
) -> float:
    """
    Retrieve angle between two points 
    * param p1: [x,y] coords of point 1
    * param p2: [x,y] coords of point 2
    * return: angle in rads
    """
    x_disp = p2[0] - p1[0]
    y_disp = p2[1] - p1[1]
    return atan2(y_disp, x_disp)


class SplineMapper(Node):
    def __init__(self, spline_len: int):
        super().__init__("spline_mapper")

        # sub to track for all cone locations relative to car start point
        self.create_subscription(Track, "/cone_pipe/track", self.map_callback, 10)

        # publishers
        self.path_marker_publisher: Publisher = self.create_publisher(Marker, "/spline_mapper/path_marker_array", 1)
        self.rh_marker_publisher: Publisher = self.create_publisher(Marker, "/spline_mapper/rh_marker_array", 1)
        self.lh_marker_publisher: Publisher = self.create_publisher(Marker, "/spline_mapper/lh_marker_array", 1)
        self.path_publisher: Publisher = self.create_publisher(SplineStamped, "/spline_mapper/path", 1)

        self.spline_len: int = spline_len
        self.track: List[Cone] = None

        LOGGER.info("---Spline Mapper Node Initalised---")


    def map_callback(self, track_msg: Track):
        LOGGER.info("Received map")
        
        start: float = time.time()
        # track cone list is taken as coords relative to the initial car position
        
        if self.track == None: self.track=track_msg.track
        elif len(self.track) == len(track_msg.track) or len(self.track) < len(track_msg.track): self.track=track_msg.track
        
        yellow_x: List[float] = []
        yellow_y: List[float] = []
        blue_x: List[float] = []
        blue_y: List[float] = []
        oranges: List[Cone] = []
        for cone in self.track:
            if cone.color == 0:
                yellow_x.append(cone.location.x)
                yellow_y.append(cone.location.y)
            elif cone.color == 1:
                blue_x.append(cone.location.x)
                blue_y.append(cone.location.y)
            elif cone.color == 4:
                oranges.append(cone)

        # 4 orange cones: 2 blue side, 2 yellow side
        for cone in oranges:
            if cone.location.x > 7: # far pair of cones
                if cone.location.y > 0: # blue side
                    blue_x.insert(0, cone.location.x)
                    blue_y.insert(0, cone.location.y)
                else: # yellow side
                    yellow_x.insert(0, cone.location.x)
                    yellow_y.insert(0, cone.location.y)
            else: # close pair of cones
                if cone.location.y > 0: # blue side
                    blue_x.append(cone.location.x)
                    blue_y.append(cone.location.y)
                else: # yellow side
                    yellow_x.append(cone.location.x)
                    yellow_y.append(cone.location.y)

        if len(yellow_x) < 4 or len(blue_x) < 4:
            #print(f"{len(yellow_x)} {len(blue_x)}")
            #print("not enough cones")
            return

        # retrieves spline lists (x,y)
        yx, yy = approximate_b_spline_path(yellow_x, yellow_y, self.spline_len)
        bx, by = approximate_b_spline_path(blue_x, blue_y, self.spline_len)

        tx: List[float] = [] # target spline x coords
        ty: List[float] = [] # target spline y coords
        th: List[float] = [] # target spline angles
        # find midpoint between splines at each point to make target path
        for i in range(self.spline_len):
            mid_x, mid_y = midpoint([yx[i], yy[i]], [bx[i], by[i]])
            tx.append(mid_x)
            ty.append(mid_y)
            # angle of tangent at midpoint
            th.append(angle([bx[i], by[i]], [yx[i], yy[i]]))

        
        ypath_markers: List[Point] = []
        ypath_colours: List[ColorRGBA] = []
        for i in range(len(yx)-1):
            line_point = Point()
            line_point.x = yx[i]
            line_point.y = yy[i]
            line_point.z = 0.0
            line_colour = ColorRGBA()
            line_colour.a = 1.0 # alpha
            line_colour.r = 0.0
            line_colour.g = 0.0
            line_colour.b = 1.0
            ypath_markers.append(line_point)
            ypath_colours.append(line_colour)

        bpath_markers: List[Point] = []
        bpath_colours: List[ColorRGBA] = []
        for i in range(len(bx)-1):
            line_point = Point()
            line_point.x = bx[i]
            line_point.y = by[i]
            line_point.z = 0.0
            line_colour = ColorRGBA()
            line_colour.a = 1.0 # alpha
            line_colour.r = 1.0
            line_colour.g = 1.0
            line_colour.b = 0.0
            bpath_markers.append(line_point)
            bpath_colours.append(line_colour)

        VEL_ZONE = 10
        path_markers: List[Point] = []
        path_colours: List[ColorRGBA] = []
        path: list[SplinePoint] = []
        for i in range(0, self.spline_len-VEL_ZONE, VEL_ZONE):
            # check angle between current and 10th spline point ahead
            th_change = th[i+VEL_ZONE] - th[i]
            # keep between 360
            if (th_change > pi): th_change=th_change-2*pi
            elif (th_change < -pi): th_change=th_change+2*pi

            # angle relative to max angle on track
            change_pc = abs(th_change) / MAX_ANGLE * 100
            # set colour proportional to angle
            col = col_range[clamp(round(change_pc), 1, 99)].get_rgb()

            for j in range(VEL_ZONE):
                path_point = SplinePoint()
                path_point.location.x = tx[i+j]
                path_point.location.y = ty[i+j]
                path_point.location.z = 0.0
                path_point.turn_intensity = change_pc
                path.append(path_point)

                line_point = Point()
                line_point.x = tx[i+j]
                line_point.y = ty[i+j]
                line_point.z = 0.0
                line_colour = ColorRGBA()
                line_colour.a = 1.0 # alpha
                line_colour.r = col[0]
                line_colour.g = col[1]
                line_colour.b = col[2]
                path_markers.append(line_point)
                path_colours.append(line_colour)

        path_msg = SplineStamped(path=path)
        self.path_publisher.publish(path_msg)

        marker = self.makeMarker(path_markers, path_colours)
        self.path_marker_publisher.publish(marker)
        rhmarker = self.makeMarker(ypath_markers, ypath_colours)
        self.rh_marker_publisher.publish(rhmarker)
        lhmarker = self.makeMarker(bpath_markers, bpath_colours)
        self.lh_marker_publisher.publish(lhmarker)


        LOGGER.info("Time taken: "+ str(time.time()-start))


    def makeMarker(self, path_markers, path_colours):
        ## Visualisation marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "current_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # scale out of 1x1x1m
        marker.scale.x = 0.2
        marker.scale.y = 0.0
        marker.scale.z = 0.0

        marker.points = path_markers
        marker.colors = path_colours

        marker.lifetime = Duration(sec=10, nanosec=100000)
        return marker

def main(args=sys.argv[1:]):
    # defaults args
    loglevel = 'info'
    print_logs = False
    spline_len = 3999

    # processing args
    opts, arg = getopt.getopt(args, str(), ['log=', 'print_logs', 'length=', 'ros-args'])

    # TODO: provide documentation for different options
    for opt, arg in opts:
        if opt == '--log':
            loglevel = arg
        elif opt == '--print_logs':
            print_logs = True
        elif opt == '--length':
            spline_len = arg
    # validating args
    numeric_level = getattr(logging, loglevel.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % loglevel)
    if not isinstance(spline_len, int):
        raise ValueError('Invalid range: %s. Must be int' % spline_len)

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

    node = SplineMapper(spline_len)
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])

