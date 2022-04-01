# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.service import Service
from rclpy.client import Client

from std_msgs.msg import StringStamped
from fs_msgs.srv import Reset

# Matrix/Array library
import numpy as np
# other python modules
import math
from math import floor, sin, cos
from typing import List
import sys
import os
import logging
import datetime
import pathlib
import threading
import time
LOGGER = logging.getLogger(__name__)

class MissionNode():
    def __init__(self, node_name, master_node: Node):
        self.node_name = node_name
        self.node_reset_client: Client = master_node.create_client(Reset, f'{self.node_name}/reset')

    def reset(self):
        self.node_reset_client.call(Reset.Request())

class MissionState(Node):
    # All variables, placed here are static

    def __init__(self):
        super().__init__("mission_state")
        self.nodes: List[MissionNode] = []
        self.pub_mission_state = self.create_publisher("mission_state", StringStamped, 10)

    def generate_mission_nodes(self):
        pass

    def send_mission_state(self, state):
        mission_state = StringStamped()
        mission_state.data = state
        mission_state.stamp = self.get_clock().now().to_msg()
        self.pub_mission_state.publish(self.mission_state)

    def check_nodes(self):
        pass



def main(args=sys.argv[1:]):
    # defaults args
    loglevel = 'info'
    print_logs = False

    numeric_level = getattr(logging, loglevel.upper(), None)

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

    node = MissionState()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    rate = node.create_rate(100)

    try:
        while rclpy.ok():
            node.check_nodes()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()

    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main(sys.argv[1:])