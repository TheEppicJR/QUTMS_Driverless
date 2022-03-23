# import ROS2 libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.publisher import Publisher
from sensor_msgs.msg import Image

# Matrix/Array library
import numpy as np
# other python modules
import math
from typing import List
import sys
import os
import logging
import datetime
import pathlib
import threading
import time

import cv2

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()

LOGGER = logging.getLogger(__name__)

class CameraNode(Node):
    # All variables, placed here are static

    def __init__(self):
        super().__init__("sr_cam")
        self.img_publisher: Publisher = self.create_publisher(Image, "/cam/sr_cam", 1)
        # open camera
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        
        if not self.cap.isOpened():
            print("Cannot open camera")
            exit()
        else:
            print("Camera connected")

    def __del__(self):
        self.cap.release()
        print('SR_CAM: Destructor called.')

    def pubImage(self):
        ret, frame = self.cap.read()
        print(type(frame))
        if frame is not None:
            self.img_publisher.publish(cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

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

    node = CameraNode()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(30)

    try:
        while rclpy.ok():
            node.pubImage()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()

    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main(sys.argv[1:])