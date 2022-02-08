# import ROS2 libraries
import imp
from turtle import st
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from cv_bridge import CvBridge
# import ROS2 message libraries
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
# import custom message libraries
from fs_msgs.msg import Track, Cone, ControlCommand

# other python modules
from math import sqrt, atan2, pi, sin, cos, atan
import cv2
import numpy as np
import matplotlib.pyplot as plt # plotting splines
import scipy.interpolate as scipy_interpolate # for spline calcs
from typing import Tuple, List
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
from .solve import MPCSolver

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

def midpoint(p1: list, p2: list):
    """
    Retrieve midpoint between two points 
    * param p1: [x,y] coords of point 1
    * param p2: [x,y] coords of point 2
    * return: x,y tuple of midpoint coord
    """
    return (p1[0]+p2[0])/2, (p1[1]+p2[1])/2


def dist(a: Point, b: Point) -> float:
    return sqrt((a.x-b.x)**2 + (a.y-b.y)**2)


def cone_to_point(cone: Cone) -> Point:
    return Point(
        cone.location.x,
        cone.location.y,
    )


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

    marker.lifetime = Duration(sec=10, nanosec=100000)

    return marker


class MPCPlanner(Node):
    def __init__(self, spline_len: int):
        super().__init__("spline_planner")

        # sub to track for all cone locations relative to car start point
        self.create_subscription(Track, "/testing_only/track", self.map_callback, 10)
        # sub to odometry for car pose + velocity
        self.create_subscription(Odometry, "/testing_only/odom", self.odom_callback, 10)

        self.mpcsolver = MPCSolver()
        
        self.mpcsolver.prepfigs()

        self.track_init = False
        # publishers
        self.plot_img_publisher: Publisher = self.create_publisher(Image, "/spline_map/plot_img", 1)
        self.path_publisher: Publisher = self.create_publisher(MarkerArray, "/spline_map/target_array", 1)
        self.control_publisher: Publisher = self.create_publisher(ControlCommand, "/control_command", 10)


        self.spline_len: int = spline_len
        self.odom_header: Header = None

        LOGGER.info("---Spline Controller Node Initalised---")



    def map_callback(self, track_msg: Track):
        LOGGER.info("Received map")
        
        # track cone list is taken as coords relative to the initial car position
        track = track_msg.track
        
        tx: List[float] = []
        ty: List[float] = []
        yellow_x: List[float] = []
        yellow_y: List[float] = []
        blue_x: List[float] = []
        blue_y: List[float] = []
        for cone in track:
            if cone.color == Cone.YELLOW:
                yellow_x.append(cone.location.x)
                yellow_y.append(cone.location.y)
            elif cone.color == Cone.BLUE:
                blue_x.append(cone.location.x)
                blue_y.append(cone.location.y)
            
        # retrieves spline lists (x,y)
        yx, yy = approximate_b_spline_path(yellow_x, yellow_y, self.spline_len)
        bx, by = approximate_b_spline_path(blue_x, blue_y, self.spline_len)
        

        # find midpoint between splines at each point to make target path
        mid_x, mid_y = [0]*self.spline_len, [0]*self.spline_len
        for i in range(self.spline_len):
            mid_x[i], mid_y[i] = midpoint([yx[i], yy[i]], [bx[i], by[i]])
            self.mpcsolver.append_track({"x" : yx[i], "y" : yy[i]},
			    {"x" : mid_x[i], "y" : mid_y[i]},
			    {"x" : bx[i], "y" : by[i]})
        self.track_init = True
        print("Init Track")


    def odom_callback(self, odom_msg: Odometry):
        # header used to create markers
        if self.track_init and self.mpcsolver.not_running:
            self.mpcsolver.not_running = True
            print("running optimisation")
            self.odom_header = odom_msg.header
            w = odom_msg.pose.pose.orientation.w
            i = odom_msg.pose.pose.orientation.x
            j = odom_msg.pose.pose.orientation.y
            k = odom_msg.pose.pose.orientation.z

            # i, j, k angles in rad
            ai, aj, ak = quat2euler([w, i, j, k])

            x = odom_msg.pose.pose.position.x
            y = odom_msg.pose.pose.position.y

            start: float = time.time()

            path_markers: List[Marker] = []

            self.mpcsolver.set_world_space(x, y, ak)

            tx, ty = self.mpcsolver.solve(None)

            for i in range(len(tx)):
                path_markers.append(marker_msg(
                    tx[i],
                    ty[i],
                    i, 
                    self.odom_header,
                ))

            LOGGER.info("Time taken: "+ str(time.time()-start))
            print("Time taken: "+ str(time.time()-start))

            # show results

            fig = self.mpcsolver.plot()
            fig.canvas.draw()

            plot_img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
            plot_img = plot_img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            plot_img = cv2.cvtColor(plot_img, cv2.COLOR_RGB2BGR)

            self.plot_img_publisher.publish(cv_bridge.cv2_to_imgmsg(plot_img, encoding="bgr8"))
            #fig.close()

            # create message for all cones on the track
            path_markers_msg = MarkerArray(markers=path_markers)
            self.path_publisher.publish(path_markers_msg)

            if len(ty) > 2:
                target_index = 2#round(len(ty) / 6) # 1/3 along
                target = Point(ty[target_index], -tx[target_index]) 
                # velocity control
                # init constants
                Kp_vel: float = 2
                vel_max: float = 8
                vel_min = vel_max/2
                throttle_max: float = 0.3 # m/s^2

                # get car vel
                vel_x: float = odom_msg.twist.twist.linear.x
                vel_y: float = odom_msg.twist.twist.linear.y
                vel: float = sqrt(vel_x**2 + vel_y**2)

                # target velocity proportional to angle
                target_vel: float = vel_max - (abs(atan(target.y / target.x))) * Kp_vel
                if target_vel < vel_min: target_vel = vel_min
                LOGGER.info(f"Target vel: {target_vel}")

                # increase proportionally as it approaches target
                throttle_scalar: float = (1 - (vel / target_vel)) 
                if throttle_scalar > 0: calc_throttle = throttle_max * throttle_scalar
                # if its over maximum, cut throttle
                elif throttle_scalar <= 0: calc_throttle = 0

                # steering control
                Kp_ang: float = 1.25
                ang_max: float = 7.0

                steering_angle = -((pi/2) - atan2(target.x, target.y))*5
                LOGGER.info(f"Target angle: {steering_angle}")
                calc_steering = Kp_ang * steering_angle / ang_max

                # publish message
                control_msg = ControlCommand()
                control_msg.throttle = float(calc_throttle)
                control_msg.steering = float(calc_steering)
                control_msg.brake = 0.0

                self.control_publisher.publish(control_msg)
            self.mpcsolver.not_running = True
        elif self.track_init and not self.mpcsolver.not_running:
            print("sim already running")
        else:
            print("Missing track")


def main(args=sys.argv[1:]):
    # defaults args
    loglevel = 'info'
    print_logs = False
    spline_len = 4000

    # processing args
    opts, arg = getopt.getopt(args, str(), ['log=', 'print_logs', 'length='])

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

    node = MPCPlanner(spline_len)
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])

