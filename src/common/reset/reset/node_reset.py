# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.service import Service
from nav_msgs.msg import Odometry
# import custom message libraries
from fs_msgs.msg import Track, Cone, ControlCommand
from fs_msgs.srv import Reset

# other python modules
from math import sqrt, atan2, pi, sin, cos, atan
from typing import Tuple, List
import time
import sys
import os
import getopt
import logging
import datetime
import pathlib

# initialise logger
LOGGER = logging.getLogger(__name__)



class ResetNode(Node):
    def __init__(self):
        super().__init__("spline_planner")

        self.req = Reset
        # self.rec.wait_on_last_task = True
        # self.svr = self.create_service(self.req, "/reset")
        print("Reset")



        LOGGER.info("---Spline Controller Node Initalised---")



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

    node = ResetNode()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])

