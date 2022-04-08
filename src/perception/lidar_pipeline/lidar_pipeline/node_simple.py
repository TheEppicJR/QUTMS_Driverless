# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# import ROS2 message libraries
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped, PointWithCovariance, PointWithCovarianceArrayStamped

# other python modules
import time
from typing import List
import sys
from math import sin, cos, sqrt, atan2
import numpy as np

# import ROS function that has been ported to ROS2 by
# SebastianGrans https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo
from .scripts.read_pcl import read_points_list
# lidar cone detection algorithm
from .scripts.sim_simple import find_cones

def cone_cov(cov, x, y, z):
    bearing = atan2(y, x)
    distance = sqrt(x**2 +y**2 + z**2)
    s, c = sin(bearing), cos(bearing)
    rotation_matrix = np.array([[c, -1*s, 0],[s, c, 0], [0, 0, 1]])
    new_cov = rotation_matrix @ cov @ rotation_matrix.T
    retcov = new_cov * (distance/20)
    return retcov

def cone_msg(x_coord: float, y_coord: float, z_coord: float) -> Cone: 
    # {Cone.YELLOW, Cone.BLUE, Cone.ORANGE_SMALL}
    location: Point = Point(
        x=x_coord,
        y=y_coord,
        z=z_coord,
    )

    return Cone(
        location=location,
        color=4,
    )

def cone_msg_cov(
    x_coord: float,
    y_coord: float,
    z_coord: float,
    colour: int,  # {Cone.YELLOW, Cone.BLUE, Cone.ORANGE_SMALL}
    cov: List[int]
) -> PointWithCovariance:

    location: Point = Point(
        x=x_coord,
        y=y_coord,
        z=z_coord,
    )

    return PointWithCovariance(
        position=location,
        color=colour,
        covariance=cov
    )
class LidarProcessing(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        self.create_subscription(PointCloud2, "/lidar/Lidar1", self.callback, 10)

        self.detection_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "lidar/cone_detection", 1)
        self.detection_publisher_cov: Publisher = self.create_publisher(PointWithCovarianceArrayStamped, "/lidar/cone_detection_cov", 1)

        self.lidarcov = np.array([[ 0.04,  0, 0], [ 0, 0.06, 0], [0, 0,  0.02]])

        self.get_logger().info('---LiDAR sim processing node initialised---')


    def callback(self, pc2_msg: PointCloud2):
        """ 
        lidar point cloud message sent here. 
        used to call funtions to find cone coords.
        to get the xyz location and colour of cones.
        """
        logger = self.get_logger()
        
        start: float = time.time()
        
        # Convert the list of floats into a list of xyz coordinates
        point_array: List[List] = read_points_list(pc2_msg)

        logger.debug("Read Time:" + str(time.time()-start))

        # calls main module from ground estimation algorithm
        cones: List[List] = find_cones(point_array) 

        logger.debug("Detected cones:" + str(len(cones)))
        
        # define message component - list of Cone type messages
        detected_cones: List[Cone] = []
        detected_cones_cov: List[PointWithCovariance] = []

        for cone in cones:
            detected_cones.append(cone_msg(cone[0], cone[1], cone[2]))
            conecov = cone_cov(self.lidarcov, cone[0], cone[1], cone[2])
            detected_cones_cov.append(cone_msg_cov(cone[0], cone[1], cone[2], 4, conecov.flatten()))
       
        detection_msg = ConeDetectionStamped(
            header=pc2_msg.header,
            cones=detected_cones
        )
        detection_msg_cov = PointWithCovarianceArrayStamped(
            header=pc2_msg.header,
            points=detected_cones_cov,
        )
        self.detection_publisher.publish(detection_msg) # publish cone data
        self.detection_publisher_cov.publish(detection_msg_cov)

        logger.debug("Total Time:" + str(time.time()-start) + "\n")


def main(args=None):
    # begin ros node
    rclpy.init(args=args)

    node = LidarProcessing()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
