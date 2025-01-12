# import ROS2 libraries
from copyreg import pickle
from re import T
from tkinter import N
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.publisher import Publisher
# import ROS2 message libraries
from nav_msgs.msg import Odometry
# import custom message libraries
from driverless_msgs.msg import SplinePoint, SplineStamped, Cone, Waypoint, WaypointsArray
from fs_msgs.msg import ControlCommand

# other python modules
from math import sqrt, atan2, sin, cos
import numpy as np
import scipy.spatial
from typing import List
import time
import sys
import os
import getopt
import logging
import datetime
import pathlib

from transforms3d.euler import quat2euler

# initialise logger
LOGGER = logging.getLogger(__name__)


def get_wheel_position(pos_cog: List[float], heading: float) -> List[float]:
    """
    Gets the position of the steering axle from the car's center of gravity and heading
    * param pos_cog: [x,y] coords of the car's center of gravity
    * param heading: car's heading in rads
    * return: [x,y] position of steering axle
    """
    #https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.1.0/vehicle_model/
    cog2axle = 0.4 #m
    x_axle = pos_cog[0] + cos(heading)*cog2axle
    y_axle = pos_cog[1] + sin(heading)*cog2axle

    return [x_axle, y_axle]


def get_RVWP(car_pos: List[float], path: np.ndarray, rvwp_lookahead: int) -> List[float]:
    """
    Retrieve angle between two points 
    * param car_pos: [x,y] coords of point 1
    * param path: [[x0,y0],[x1,y1],...,[xn-1,yn-1]] path points
    * param rvwpLookahead: how many indices to look ahead in path array for RVWP
    * return: RVWP position as [x,y]
    """
    _pos = np.array([[car_pos[0], car_pos[1]]])
    dists: np.ndarray = scipy.spatial.distance.cdist(path,_pos, 'euclidean')
    min_index: int = np.where(dists == np.amin(dists))[0][0]

    rvwp_index: int = (min_index + rvwp_lookahead) % len(path)
    rvwp: List[float] = path[rvwp_index]

    return rvwp


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


def wrap_to_pi(angle: float) -> float:
    """
    Wrap an angle between -pi and pi
    * param angle: angle in rads
    * return: angle in rads wrapped to -pi and pi
    """

    # https://stackoverflow.com/a/15927914/12206202
    return (angle + np.pi) % (2 * np.pi) - np.pi


def get_throttle_and_brake(velocities: List[float], steering_angle: float) -> List[float]:
    """
    Decrease velocity proportional to the desired steering angle
    * param velocities: [x,y] current x & y velocities
    * param steering_angle: the angle the car needs to turn
    * return: [calc_throttle, calc_break]
    """
    # velocity control
    # init constants
    Kp_vel: float = 20
    vel_max: float = 15
    vel_min = 1
    throttle_max: float = 0.5 # m/s^2
    brake_max = 0.25

    # get car vel
    vel_x: float = velocities[0]
    vel_y: float = velocities[1]
    vel: float = sqrt(vel_x**2 + vel_y**2)
    
    # target velocity proportional to angle
    target_vel: float = vel_max - abs(steering_angle) * Kp_vel
    if target_vel < vel_min: target_vel = vel_min

    # increase proportionally as it approaches target
    throttle_scalar: float = (1 - (vel / target_vel)) 
    calc_brake = 0.0
    if throttle_scalar > 0: 
        calc_throttle = throttle_max * throttle_scalar
    # if its over maximum, brake propotionally unless under minimum
    else:
        calc_throttle = 0
        if (vel > vel_min):
            calc_brake = abs(brake_max * throttle_scalar)

    return [calc_throttle, calc_brake]


class SplinePursuit(Node):
    def __init__(self):
        super().__init__("spline_planner")

        # sub to path mapper for the desired vehicle path (as an array)
        self.create_subscription(SplineStamped, "/spline_mapper/path", self.path_callback, 10)
        # sub to odometry for car pose + velocity
        self.create_subscription(Odometry, "/testing_only/odom", self.callback, 10)
        
        # publishers
        self.control_publisher: Publisher = self.create_publisher(ControlCommand, "/control_command", 10)

        # path is a numpy array with 2 dimensions
        self.path: np.ndarray = None


        self.path = None
        self.last_target_idx = 3

        LOGGER.info("---Spline Controller Node Initalised---")

    def path_callback(self, spline_path_msg: SplineStamped):
        # Only set the desired path once (before the car is moving)
        if self.path is not None: return

        # convert List[SplinePoint] to 2D numpy array
        start: float = time.time()
        self.path = np.array([[p.location.x, p.location.y] for p in spline_path_msg.path])
        LOGGER.info("Time taken to convert to np array: "+ str(time.time()-start))
        LOGGER.info(f"Spline Path Recieved - length: {len(self.path)}")

    def callback(self, odom_msg: Odometry):
        # Only start once the path has been recieved
        if self.path is None: return

        w = odom_msg.pose.pose.orientation.w
        i = odom_msg.pose.pose.orientation.x
        j = odom_msg.pose.pose.orientation.y
        k = odom_msg.pose.pose.orientation.z

        # i, j, k angles in rad
        ai, aj, ak = quat2euler([w, i, j, k])
        heading: float = ak

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        # get the position of the center of gravity
        position_cog: List[float] = [x, y]
        position: List[float] = get_wheel_position(position_cog, heading)
        
        # rvwp control
        rvwpLookahead = 75

        rvwp: List[float] = get_RVWP(position, self.path, rvwpLookahead)

        # steering control
        des_heading_ang: float = angle(position, rvwp)
        steering_angle: float = wrap_to_pi(heading - des_heading_ang)

        calc_steering = steering_angle

        vel: List[float] = [odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y]
        [calc_throttle, calc_break] = get_throttle_and_brake(vel, steering_angle)

        # publish message
        control_msg = ControlCommand()
        control_msg.steering = float(calc_steering)
        control_msg.throttle = float(calc_throttle)
        control_msg.brake = float(calc_break)

        self.control_publisher.publish(control_msg)


def main(args=sys.argv[1:]):
    # defaults args
    loglevel = 'info'
    print_logs = False

    # processing args
    opts, arg = getopt.getopt(args, str(), ['log=', 'print_logs', 'ros-args'])

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

    node = SplinePursuit()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()

    
if __name__ == '__main__':
    main(sys.argv[1:])

