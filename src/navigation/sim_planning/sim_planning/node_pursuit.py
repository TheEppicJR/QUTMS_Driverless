# import ROS2 libraries
from re import T
from tkinter import N
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.publisher import Publisher
from cv_bridge import CvBridge
# import ROS2 message libraries
from nav_msgs.msg import Odometry
# import custom message libraries
from driverless_msgs.msg import SplinePoint, SplineStamped, Cone
from fs_msgs.msg import ControlCommand

# other python modules
from math import sqrt, atan2, pi, sin, cos, atan
import cv2
import numpy as np
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
from driverless_common.point import Point

# initialise logger
LOGGER = logging.getLogger(__name__)

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()

# image display geometry
SCALE = 20
WIDTH = 40*SCALE # 10m either side
HEIGHT = 40*SCALE # 20m forward
ORIGIN = Point(0, 0)
IMG_ORIGIN = Point(int(WIDTH/2), int(HEIGHT/2))

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

def angle_between_angles(theta: float, phi: float, spread: float) -> bool:
    a, b = lim2pi(phi - spread), lim2pi(phi + spread)
    if a < b and theta > a and theta < b:
        return True
    if a < b and (theta < a or theta > b):
        return True
    return False

def lim2pi(theta: float) -> float:
    if (theta > pi): phi=theta-2*pi
    elif (theta < -pi): phi=theta+2*pi
    else: phi = theta
    return phi

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



class SplinePursuit(Node):
    def __init__(self, spline_len: int):
        super().__init__("spline_planner")

        # sub to track for all cone locations relative to car start point
        self.create_subscription(SplineStamped, "/spline_mapper/path", self.path_callback, 10)
        self.path_img_publisher: Publisher = self.create_publisher(Image, "/pursuit/path_img", 1)

        # sub to odometry for car pose + velocity
        self.create_subscription(Odometry, "/testing_only/odom", self.callback, 10)
        self.control_publisher: Publisher = self.create_publisher(ControlCommand, "/control_command", 10)


        self.path = None

        LOGGER.info("---Spline Controller Node Initalised---")


    def path_callback(self, spline_path: SplineStamped):
        self.path = spline_path.path

    def callback(self, odom_msg: Odometry):

        # target spline markers for rviz
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        w = odom_msg.pose.pose.orientation.w
        i = odom_msg.pose.pose.orientation.x
        j = odom_msg.pose.pose.orientation.y
        k = odom_msg.pose.pose.orientation.z

        # i, j, k angles in rad
        ai, aj, ak = quat2euler([w, i, j, k])

        debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        ## APPROACH TARGET
        if self.path is not None:
            target: Point = None

            for tpoint in self.path:
                dx, dy = x-tpoint.location.x, y-tpoint.location.y
                angle = atan2(dy , dx)
                #print(f"Point: {tpoint.location.x}, {tpoint.location.y} Car: {x}, {y}, {ak/2/pi*360} Vector: {angle/2/pi*360}")
                if angle_between_angles(angle, ak, pi/6) and target is None:
                    target = tpoint.location
            if target is None:
                target = self.path[-1].location

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
            target_vel: float = vel_max - (abs(atan2(target.y, target.x))) * Kp_vel
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

            # draw target
            target_img_pt = robot_pt_to_img_pt(target.x, target.y)
            cv2.drawMarker(
                debug_img, 
                robot_pt_to_img_pt(target.x, target.y).to_tuple(),
                (0, 0, 255),
                markerType=cv2.MARKER_TILTED_CROSS,
                markerSize=10,
                thickness=2
            )
            target_img_angle = atan2(target_img_pt.y - IMG_ORIGIN.y, target_img_pt.x - IMG_ORIGIN.x)
            # draw angle line
            cv2.line(
                debug_img,
                (int(50*cos(target_img_angle) + IMG_ORIGIN.x), int(50*sin(target_img_angle) + IMG_ORIGIN.y)),
                IMG_ORIGIN.to_tuple(),
                (0, 0, 255)
            )
            # add text for targets data
            cv2.putText(
                debug_img, "Targets", (10, HEIGHT-40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
            )
            text_angle = "Steering: "+str(round(steering_angle, 2))
            cv2.putText(
                debug_img, text_angle, (10, HEIGHT-25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
            )
            text_vel = "Velocity: "+str(round(target_vel, 2))
            cv2.putText(
                debug_img, text_vel, (10, HEIGHT-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
            )

        self.path_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))


def main(args=sys.argv[1:]):
    # defaults args
    loglevel = 'info'
    print_logs = False
    spline_len = 200

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

    node = SplinePursuit(spline_len)
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])

