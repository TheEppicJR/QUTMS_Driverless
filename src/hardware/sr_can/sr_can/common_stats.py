
# import ROS2 libraries
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time, Duration

# import ROS2 message libraries
from driverless_msgs.msg import GenericEnum, GenericSensor
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped

from sensor_msgs.msg import Imu, MagneticField

from typing import List, Dict
import sys
import os
import logging
import datetime
import pathlib

from transforms3d.euler import quat2euler, euler2quat


class Common_Stats(Node):
    # All variables, placed here are static

    def __init__(self):
        super().__init__("sr_can")

        self.heading_pub: Publisher = self.create_publisher(Vector3Stamped, "/processed/roll_pitch_yaw", 1)
        self.gg_pub: Publisher = self.create_publisher(Vector3Stamped, "/processed/gg", 1)
        self.create_subscription(Imu, "/sr_imu/imu_acc_ar", self.imu_callback, 10)

        #self.magheading_pub: Publisher = self.create_publisher(Vector3Stamped, "/processed/mag_roll_pitch_yaw", 1)
        #self.create_subscription(Imu, "/cone_pipe/cone_detection_cov", self.imu_callback, 10)

        print("Common_Stats Constructor has been called")

    def __del__(self):
        print('Common_Stats: Destructor called.')

    def imu_callback(self, imu_reading: Imu):
        heading_msg = Vector3Stamped()
        heading_msg.header = imu_reading.header
        (heading_msg.vector.x, heading_msg.vector.y, heading_msg.vector.z) = quat2euler([imu_reading.orientation.w, imu_reading.orientation.x, imu_reading.orientation.y, imu_reading.orientation.z])
        self.heading_pub.publish(heading_msg)
        gg_msg = Vector3Stamped()
        gg_msg.header = imu_reading.header
        gg_msg.vector.x, gg_msg.vector.y, gg_msg.vector.z = imu_reading.linear_acceleration.x/9.81, imu_reading.linear_acceleration.y/9.81, imu_reading.linear_acceleration.z/9.81
        self.gg_pub.publish(gg_msg)
        

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

    rclpy.init(args=args)

    node = Common_Stats()

    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])