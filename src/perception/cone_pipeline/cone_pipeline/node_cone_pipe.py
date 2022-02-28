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
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
# import custom message libraries
from driverless_msgs.msg import Cone as QUTCone
from driverless_msgs.msg import ConeDetectionStamped, PointWithCovarianceStamped, PointWithCovarianceStampedArray

from typing import List


# other python modules
import numpy as np
import sys
import os
import getopt
import logging
import pathlib

from transforms3d.euler import quat2euler

# import required sub modules
from .point import PointWithCov
from . import kmeans_clustering as km
from .kdtree import create, KDNode
from .kdtree import Node as kdNode

# initialise logger
LOGGER = logging.getLogger(__name__)

LEFT_CONE_COLOUR = QUTCone.BLUE
RIGHT_CONE_COLOUR = QUTCone.YELLOW

class ConePipeline(Node):
    def __init__(self):
        super().__init__("cone_pipeline")

        self.logger = self.get_logger()

        self.create_subscription(PointWithCovarianceStampedArray, "/lidar/cone_detection_cov", self.lidarCallback, 10) # "/lidar/cone_detection_cov"
        self.create_subscription(PointWithCovarianceStampedArray, "/vision/cone_detection_cov", self.visionCallback, 10) # "/detector/cone_detection_cov"

        self.filtered_cones_pub: Publisher = self.create_publisher(ConeDetectionStamped, "/cone_pipe/cone_detection", 1)
        self.filtered_cones_pub_cov: Publisher = self.create_publisher(PointWithCovarianceStampedArray, "/cone_pipe/cone_detection_cov", 1)

        self.lidar_markers: Publisher = self.create_publisher(MarkerArray, "/cone_pipe/lidar_marker", 1)
        self.vision_markers: Publisher = self.create_publisher(MarkerArray, "/cone_pipe/vision_marker", 1)
        self.filtered_markers: Publisher = self.create_publisher(MarkerArray, "/cone_pipe/filtered_marker", 1)

        odom_sub = message_filters.Subscriber(self, Odometry, "/odometry/global") # "/testing_only/odom"
        self.actualodom = message_filters.Cache(odom_sub, 1000) # needs to be the more than the max latency of perception in ms

        self.printmarkers: bool = True

        self.conesKDTree: KDNode = None

        self.bufferKDTree: KDNode = None

        self.z_datum: float = 0

        # this is really the wrong way to do this but I just need a solution for now
        self.z_datum_avg: float = 0
        self.z_readings: float = 1

        LOGGER.info("---Cone Pipeline Node Initalised---")
        self.logger.debug("---Cone Pipeline Node Initalised---")

    def getNearestOdom(self, stamp):
        # need to switch this over to a position from a EKF with covariance
        locodom: Odometry = self.actualodom.getElemAfterTime(Time.from_msg(stamp))
        #cov = np.identity(6) * 0.0025 # standin covariance for ekf assuming the variance is sigma = 5cm with no covariance
        # if the nearest Odom in the cache is more than 0.05 sec off then just throw it away
        if locodom is not None:
            #print(Time.from_msg(stamp) - Time.from_msg(locodom.header.stamp))
            if Time.from_msg(stamp) - Time.from_msg(locodom.header.stamp) > Duration(nanoseconds=0.05*(10**9)):
                return None
        return locodom

    def fuseCone(self, point):
        # find the closest cone in the cone tree
        closestcone = self.conesKDTree.search_knn(point, 1)
        # if its close enough to a actual cone than fuse it and rebalance in case it moved a bit too much (should only really matter for the orange cones near the start and may not need to rebalance at this step but why not) (im sure i will remove the rebalance to spare my cpu later and then the whole thing will break lol)
        if len(closestcone) > 0 and closestcone[0][0].data.inFourSigma(point):
            closestcone[0][0].data.update(point)
            self.conesKDTree.rebalance()
        # otherwise check it against the buffer
        else:
            self.bufferCone(point)
            
    def updateZDatum(self, zHeight: float):
        self.z_readings += 1
        self.z_datum_avg = (self.z_datum_avg*(self.z_readings-1)+zHeight)/self.z_readings

    def bufferCone(self, point):
        # if we already have a list of possible cones then look through that tree for something close
        if self.bufferKDTree is not None and self.bufferKDTree.data is not None:
            # find the closest possible cone to the possible cone
            closestcone = self.bufferKDTree.search_knn(point, 1)
            # see if it is close enough for our liking (ths distance in m) than we will turn it into a actual cone
            if closestcone[0][0].data.inTwoSigma(point):
                # select the first group from the returned tuple (point objects) and then get the first one (which will be our point since we only asked for one)
                pointnew: kdNode = closestcone[0][0]
                # fuse the points together
                pointnew.data.update(point)
                # if there is already a tree of Offical Cones tm than add it to that tree and then rebalance it
                if self.conesKDTree is not None:
                    self.conesKDTree.add(pointnew.data)
                    self.conesKDTree.rebalance()
                # if not than make a tree for them
                else:
                    self.conesKDTree = create([point])
                # remove the point from the buffer tree
                self.bufferKDTree.remove(pointnew.data)
                # the rebalance the buffer tree since we removed something
                self.bufferKDTree.rebalance()
            # if nothind is close in the buffer than add it to the buffer tree and rebalance
            else:
                self.bufferKDTree.add(point)
                self.bufferKDTree.rebalance()
        # if we dont already have a buffer tree than create one (this should only ever happen once i hope, otherwise something has gone horrible wrong)
        else:
            self.bufferKDTree = create([point])

    def fusePoints(self, points):
        # if we have cones in the cone tree than check if we can fuse our new points with them
        if self.conesKDTree is not None:
            for point in points:
                self.fuseCone(point)
        # if not we will check them against the buffer (or create a buffer) this should happen the tirst two cycles the fuse points is run (first there wont be either tree and the second time there wont be a cone tree yet)
        else:
            for point in points:
                self.bufferCone(point)
        # if we have cones we are sure about than send them out in messages
        if self.conesKDTree is not None and self.conesKDTree.data is not None:
            # make a bool to determine if we are going to spend the time to generate markers
            msgs = self.printmarkers and self.filtered_markers.get_subscription_count() > 0
            # get all the cone elements from the tree structure
            curcones = self.conesKDTree.returnElements()
            # create the lists to fill with our elements
            conelist: List[QUTCone] = []
            conelist_cov: List[PointWithCovarianceStamped] = []
            markers: List[Marker] = []
            msgid = 0
            for cone in curcones:
                # should probabbly put a filter for how many times a cone has actually been spotted
                if cone.covMax(0.5):
                    # create out messages to be published with the final cones that we found
                    pubpt_cov = PointWithCovarianceStamped()
                    pubpt = QUTCone()
                    # create a point at the location of the cone
                    point = Point()
                    point.x = cone.global_x
                    point.y = cone.global_y
                    point.z = cone.global_z
                    # create the row major 3x3 cov matrix for the message elements
                    pubpt_cov.covariance = cone.global_cov.flatten()
                    # set those parts of the message
                    pubpt_cov.position = point
                    pubpt_cov.header = cone.header
                    pubpt_cov.color = cone.color
                    pubpt.location = point
                    # set its color
                    pubpt.color = cone.color

                    # append those elements to the list of elements for the message
                    #if True:#cone.color < 4:
                    conelist.append(pubpt)
                    conelist_cov.append(pubpt_cov)

                    if msgs:
                        markers.append(cone.getCov(msgid, False, self.z_datum_avg))
                        markers.append(cone.getMarker(msgid+1, self.z_datum_avg))
                    msgid += 2
            if msgs:
                mkr = MarkerArray()
                mkr.markers = markers
                self.filtered_markers.publish(mkr)
            
            # create a ConeDetectionStamped message
            coneListPub = ConeDetectionStamped()
            coneListPub.cones = conelist
            # idk what makes sense for setting the header on this detection, this is probably wrong but idc atm
            coneListPub.header = curcones[0].header
            # create a PointWithCovarianceStampedArray message
            coneListPubCov = PointWithCovarianceStampedArray()
            coneListPubCov.points = conelist_cov
            # publish the messages
            self.filtered_cones_pub.publish(coneListPub)
            self.filtered_cones_pub_cov.publish(coneListPubCov)

        # need to make a section to remove old points from the buffer
        if self.bufferKDTree is not None and self.bufferKDTree.data is not None:
            for point in self.bufferKDTree.returnElements():
                if self.get_clock().now() - Duration(nanoseconds=2*10**9) > Time.from_msg(point.header.stamp):
                    self.bufferKDTree.remove(point)
            self.bufferKDTree.rebalance()
        if self.conesKDTree is not None and self.conesKDTree.data is not None:
            for point in self.conesKDTree.returnElements():
                knn = self.conesKDTree.search_knn(point, 2)
                #print(point.global_z)
                if len(knn) > 1:
                    if point.inFourSigma(knn[1][0].data) and (point.color == knn[1][0].data.color or (point.color == 4 or knn[1][0].data.color == 4)):
                        knn[1][0].data.update(point)
                        self.conesKDTree.remove(point)
                        self.conesKDTree.rebalance()
                    elif (point.nMeasurments > 3 and point.covMin(1)) or point.global_z-self.z_datum < -0.3 or point.global_z-self.z_datum > 1.0: # 0.1 and 0.6 for sim
                        self.conesKDTree.remove(point)
                        self.conesKDTree.rebalance()
                if point.global_z-self.z_datum < -0.3 or point.global_z-self.z_datum > 1.0: # 0.1 and 0.6 for sim
                    print(f"X: {point.global_x}, Y: {point.global_y}, Z: {point.global_z}, Z_datum: {self.z_datum}, dZ: {point.global_z-self.z_datum}")
                    self.conesKDTree.remove(point)
                    self.conesKDTree.rebalance()

            
        
    def lidarCallback(self, points: PointWithCovarianceStampedArray):
        if len(points.points) > 0:
            msgs = self.printmarkers and self.lidar_markers.get_subscription_count() > 0
            header = points.points[0].header
            odomloc = self.getNearestOdom(header.stamp)
            
            if odomloc is None:
                return None
            
            self.z_datum = odomloc.pose.pose.position.z
            self.updateZDatum(self.z_datum)

            conelist: List[PointWithCov] = []
            markers: List[Marker] = []
            msgid = 0
            for point in points.points:
                p = PointWithCov(point.position.x, point.position.y, point.position.z, np.array(point.covariance).reshape((3,3)), 4, point.header)
                p.translate(odomloc)
                if point.position.z < 0.45 and point.position.z > 0.15:
                    conelist.append(p)
                    if msgs:
                        markers.append(p.getCov(msgid, True, self.z_datum_avg))
                        markers.append(p.getMarker(msgid+1, self.z_datum_avg))
                    msgid += 2
            if msgs:
                mkr = MarkerArray()
                mkr.markers = markers
                self.lidar_markers.publish(mkr)
            self.fusePoints(conelist)

    def visionCallback(self, points: PointWithCovarianceStampedArray):
        if len(points.points) > 0:
            msgs = self.printmarkers and self.vision_markers.get_subscription_count() > 0
            header = points.points[0].header
            odomloc= self.getNearestOdom(header.stamp)
            if odomloc is None:
                return None

            self.z_datum = odomloc.pose.pose.position.z
            self.updateZDatum(self.z_datum)
            
            conelist: List[PointWithCov] = []
            markers: List[Marker] = []
            msgid = 0
            for point in points.points:
                p = PointWithCov(point.position.x, point.position.y, point.position.z, np.array(point.covariance).reshape((3,3)), point.color, point.header)
                p.translate(odomloc)
                conelist.append(p)
                if msgs:
                    markers.append(p.getCov(msgid, True, self.z_datum_avg))
                    markers.append(p.getMarker(msgid + 1, self.z_datum_avg))
                msgid += 2
            if msgs:
                mkr = MarkerArray()
                mkr.markers = markers
                self.vision_markers.publish(mkr)
            self.fusePoints(conelist)



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

