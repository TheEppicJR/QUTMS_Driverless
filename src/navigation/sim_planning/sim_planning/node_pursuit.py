# import ROS2 libraries
from copyreg import pickle
from re import T
from tkinter import N
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.publisher import Publisher
from geometry_msgs.msg import Point as ROSPoint
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
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



def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return 

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
    if a > b and (theta < a or theta > b):
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
    x_list[1] = x #+ [0.0, 0.0, 0.0, 0.0]
    y_list[1] = y #+ [0.0, 0.0, 0.0, 0.0]

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
        self.path_marker_publisher: Publisher = self.create_publisher(Marker, "/local_spline/path_marker", 1)
        self.target_marker_publisher: Publisher = self.create_publisher(Marker, "/local_spline/target_marker", 1)


        self.path = None
        self.last_target_idx = 0

        LOGGER.info("---Spline Controller Node Initalised---")


    def path_callback(self, spline_path: SplineStamped):
        self.path = spline_path.path
        self.last_target_idx = 0

    def callback(self, odom_msg: Odometry):

        # target spline markers for rviz
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        vel_x: float = odom_msg.twist.twist.linear.x
        vel_y: float = odom_msg.twist.twist.linear.y
        vel: float = sqrt(vel_x**2 + vel_y**2)
        w = odom_msg.pose.pose.orientation.w
        i = odom_msg.pose.pose.orientation.x
        j = odom_msg.pose.pose.orientation.y
        k = odom_msg.pose.pose.orientation.z

        # i, j, k angles in rad
        ai, aj, ak = quat2euler([w, i, j, k])

        debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        path_markers: List[Marker] = []

        ## APPROACH TARGET
        if self.path is not None:
            target: Point = None
            slow = False
            L = 2
            fx = x + L * np.cos(ak)
            fy = y + L * np.sin(ak)

            cx: List[float] = []
            cy: List[float] = []
            for tpoint in self.path:
                dx, dy = tpoint.location.x - x, tpoint.location.y - y
                cx.append(tpoint.location.x)
                cy.append(tpoint.location.y)
                angle = atan2(dy , dx)
                #print(f"Point: {tpoint.location.x}, {tpoint.location.y} Car: {x}, {y}, {ak/2/pi*360} Vector: {angle/2/pi*360}")
                if angle_between_angles(angle, ak, pi/6) and target is None:
                    target = tpoint.location
                    slow = True
                    print("found one")
                line_point = ROSPoint()
                line_point.x = tpoint.location.x-x
                line_point.y = tpoint.location.y-y
                line_point.z = 0.0
                path_markers.append(line_point)
            if target is None:
                target = self.path[-1].location
                print("didnt find one in front")
            
            # Search nearest point index
            dx = [fx - icx for icx in cx]
            dy = [fy - icy for icy in cy]
            d = np.hypot(dx, dy)
            target_idx = np.argmin(d)
            if self.last_target_idx >= target_idx:
                target_idx = self.last_target_idx
            front_axle_vec = [-np.cos(ak + np.pi / 2),-np.sin(ak + np.pi / 2)]
            error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)
            # theta_e corrects the heading error
            theta_e = normalize_angle(atan2(dy[target_idx],dx[target_idx]) - ak)
            # theta_d corrects the cross track error
            theta_d = np.arctan2(k * error_front_axle, vel)
            # Steering control
            #delta = theta_e + theta_d

            target = self.path[target_idx].location
            print(target)
            print(f"{x}    {y}  {target_idx}")

            # Project RMS error onto front axle vector
            front_axle_vec = [-np.cos(ak + np.pi / 2),
                      -np.sin(ak + np.pi / 2)]
            error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

            # velocity control
            # init constants
            Kp_vel: float = 2
            vel_max: float = 8
            vel_min = vel_max/2
            throttle_max: float = 0.3 # m/s^2

            

            #if slow:
            #    vel_max = 0.5
                
            
            # target velocity proportional to angle
            target_vel: float = vel_max - (abs(atan2(y-target.y, x-target.x))) * Kp_vel
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

            steering_angle = -((pi/2) - atan2(x-target.x, y-target.y))*5
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

            # add on each cone to published array
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "current_path"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.pose.position = odom_msg.pose.pose.position
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            # scale out of 1x1x1m
            marker.scale.x = 0.2
            marker.scale.y = 0.0
            marker.scale.z = 0.0

            marker.points = path_markers

            marker.color.a = 1.0 # alpha
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker.lifetime = Duration(sec=1, nanosec=0)

            # create message for all cones on the track
            self.path_marker_publisher.publish(marker) # publish marker points data

            target_markers: List[Marker] = []
            # line_point2 = ROSPoint()
            # line_point2.x = target.x
            # line_point2.y = target.y
            # line_point2.z = 0.0
            target_markers.append(target)#line_point2)
            print(f"Target again {target}")

            marker2 = Marker()
            marker2.header.frame_id = "map"
            marker2.ns = "current_path"
            marker2.id = 0
            marker2.type = Marker.SPHERE
            marker2.action = Marker.ADD

            marker2.pose.position = target
            marker2.pose.orientation.x = 0.0
            marker2.pose.orientation.y = 0.0
            marker2.pose.orientation.z = 0.0
            marker2.pose.orientation.w = 1.0
            # scal2e out of 1x1x1m
            marker2.scale.x = 0.2
            marker2.scale.y = 0.2
            marker2.scale.z = 0.2

            marker2.points = target_markers

            marker2.color.a = 1.0 # alpha
            marker2.color.r = 0.0
            marker2.color.g = 1.0
            marker2.color.b = 0.0
            marker2.lifetime = Duration(sec=1, nanosec=0)

            self.target_marker_publisher.publish(marker2)

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

