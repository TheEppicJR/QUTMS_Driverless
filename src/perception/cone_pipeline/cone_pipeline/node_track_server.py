# !NUT
# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.clock import ClockType
from rclpy.time import Time, Duration
import rclpy.logging
import message_filters
# import ROS2 message libraries
from std_msgs.msg import Header
from geometry_msgs.msg import Point as PointMsg
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Duration as DurationMsg
# import custom message libraries
from driverless_msgs.msg import Cone as QUTCone
from driverless_msgs.msg import ConeDetectionStamped, PointWithCovarianceStamped, PointWithCovarianceStampedArray

from typing import List

# other python modules
import math
import numpy as np
import sys
import os
import getopt
import logging
import pathlib

from scipy.spatial import Delaunay

# import required sub modules
from .point import PointWithCov, Edge, Triangle, Point
from .kdtree import create, KDNode

# initialise logger
LOGGER = logging.getLogger(__name__)


class ConePipeline(Node):
    def __init__(self):
        super().__init__("cone_pipeline")

        self.logger = self.get_logger()

        self.create_subscription(PointWithCovarianceStampedArray, "/cone_pipe/cone_detection_cov", self.mapCallback, 10) 

        self.track_markers: Publisher = self.create_publisher(MarkerArray, "/cone_pipe/track_marker", 1)

        self.delaunayLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/delaunay_lines", 1)
        self.leftLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/left_line", 1)
        self.rightLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/right_line", 1)
        self.startLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/start_line", 1)
        self.delLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/del_line", 1)
        self.qsLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/qs_line", 1)

        odom_sub = message_filters.Subscriber(self, Odometry, "/testing_only/odom")
        self.actualodom = message_filters.Cache(odom_sub, 1000) # needs to be the more than the max latency of perception in ms

        self.printmarkers: bool = True

        LOGGER.info("---Cone Pipeline Node Initalised---")
        self.logger.debug("---Cone Pipeline Node Initalised---")

    def mapCallback(self, track_msg: PointWithCovarianceStampedArray):
        self.map = track_msg.points
        quecones = self.getDelaunayEdges(self.map)
        #self.publishDelaunayEdgesVisual()
        if quecones:
            self.getEdges()


    def orderLines(self, lines):
        usedLines = []
        orderedLines = []
        for i in range(len(lines)):
            line = lines[i]
            if line not in usedLines:
                pass

    def getEdges(self):
        usedEdges = []
        leftHandEdges: List[Edge] = []
        rightHandEdges: List[Edge] = []
        discardedEdges: List[Edge] = []
        startingLineEdges: List[Edge] = []
        qsLineEdges: List[Edge] = []
        for triangle in self.triangleList:
            for edge in triangle.getEdges():
                if not edge.calledFor:
                    x, y = edge.getMiddlePoint()
                    ne: Edge = self.edgeKDTree.search_knn(Point(x, y), 1)[0][0].data
                    if not ne.calledFor:
                        if edge.length() > 10: discardedEdges.append(edge)
                        elif edge.color == 0: leftHandEdges.append(edge)
                        elif edge.color == 1: rightHandEdges.append(edge)
                        elif edge.color == 2 or edge.color == 3: qsLineEdges.append(edge)
                        elif edge.color == 4: startingLineEdges.append(edge)
                        else: discardedEdges.append(edge)
                        edge.calledFor = True
                        ne.calledFor = True
                    
        leftHandMsg = self.trackEdgesVisual(leftHandEdges, 0)
        rightHandMsg = self.trackEdgesVisual(rightHandEdges, 1)
        discardedEdgesMsg = self.trackEdgesVisual(discardedEdges, 5)
        startingEdgesMsg = self.trackEdgesVisual(startingLineEdges, 3)
        qsEdgesMsg = self.trackEdgesVisual(qsLineEdges, 2)
        allEdgesMsg = self.trackEdgesVisual(self.edgeList, 5)

        if leftHandMsg is not None:
            self.leftLinesVisualPub.publish(leftHandMsg)
        if rightHandMsg is not None:
            self.rightLinesVisualPub.publish(rightHandMsg)
        if discardedEdgesMsg is not None:
            self.delLinesVisualPub.publish(discardedEdgesMsg)
        if startingEdgesMsg is not None:
            self.startLinesVisualPub.publish(startingEdgesMsg)
        if qsEdgesMsg is not None:
            self.qsLinesVisualPub.publish(qsEdgesMsg)
        if allEdgesMsg is not None:
            self.delaunayLinesVisualPub.publish(allEdgesMsg)


    def getDelaunayEdges(self, frontCones):
        if len(frontCones) < 4: # no sense to calculate delaunay
            return False

        conePoints = np.zeros((len(frontCones), 2))
        coneList: List[PointWithCov] = []
        triangleList: List[Triangle] = []
        edgeList: List[Edge] = []

        for i in range(len(frontCones)):
            cone = frontCones[i]
            conePoints[i] = ([cone.position.x, cone.position.y])
            coneList.append(PointWithCov(0, 0, 0, None, cone.color, cone.header, cone.position.x, cone.position.y, cone.position.z, np.array(cone.covariance).reshape((3,3))))

        tri = Delaunay(conePoints)
        self.coneKDTree = create(coneList)
        
        for simp in tri.simplices:
            p1 = self.coneKDTree.search_knn(Point(conePoints[simp[0]][0], conePoints[simp[0]][1]), 1)[0][0].data
            p2 = self.coneKDTree.search_knn(Point(conePoints[simp[1]][0], conePoints[simp[1]][1]), 1)[0][0].data
            p3 = self.coneKDTree.search_knn(Point(conePoints[simp[2]][0], conePoints[simp[2]][1]), 1)[0][0].data
            edgeList.append(Edge(p1, p2))
            edgeList.append(Edge(p2, p3))
            edgeList.append(Edge(p3, p1))
            triangleList.append(Triangle(p1, p2, p3))

        self.triangleList: List[Triangle] = triangleList
        self.edgeList: List[Edge] = edgeList
        self.edgeKDTree = create(edgeList)
        self.triangleKDTree = create(triangleList)
        return True


    def trackEdgesVisual(self, edges: List[Edge], cc: int):
        if not edges:
            return None

        marker = Marker()
        marker.header.frame_id = "map"
        marker.lifetime = DurationMsg(sec=1)
        marker.ns = "publishTrackLinesVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05

        marker.pose.orientation.w = 1.0

        marker.color.a = 0.5
        if cc == 0:
            marker.color.b = 1.0
        elif cc == 1:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif cc == 2 or cc == 3 or cc == 4:
            marker.color.r = 1.0
            marker.color.g = 0.7
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        path_markers: List[PointMsg] = []
        for edge in edges:
            p1, p2 = edge.getPointMsg()
            path_markers.append(p1)
            path_markers.append(p2)
        marker.points = path_markers
        return marker
        

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

    date = "hi" # Hi!
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

    node = ConePipeline()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])