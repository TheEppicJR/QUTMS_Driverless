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
from geometry_msgs.msg import Point, Twist, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
# import custom message libraries
from driverless_msgs.msg import Cone as QUTCone
from driverless_msgs.msg import ConeDetectionStamped, PointWithCovariance, PointWithCovarianceArrayStamped

from typing import List

# import tf2
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# other python modules
import numpy as np
import sys
import os
import getopt
import logging
import pathlib

from transforms3d.euler import quat2euler

# import required sub modules
from .point import PointWithCov, do_transform_point
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

        self.declare_parameter('target_frame', 'fsds/FSCar')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.br = TransformBroadcaster(self)
        self._tf_publisher = StaticTransformBroadcaster(self)
        self.odom_callback()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PointWithCovarianceArrayStamped, "/lidar/cone_detection_cov", self.lidarCallback, 10)
        self.create_subscription(PointWithCovarianceArrayStamped, "/vision/cone_detection_cov", self.visionCallback, 10)

        self.filtered_cones_pub: Publisher = self.create_publisher(ConeDetectionStamped, "/cone_pipe/cone_detection", 1)
        self.filtered_cones_pub_cov: Publisher = self.create_publisher(PointWithCovarianceArrayStamped, "/cone_pipe/cone_detection_cov", 1)

        self.lidar_markers: Publisher = self.create_publisher(MarkerArray, "/cone_pipe/lidar_marker", 1)
        self.vision_markers: Publisher = self.create_publisher(MarkerArray, "/cone_pipe/vision_marker", 1)
        self.filtered_markers: Publisher = self.create_publisher(MarkerArray, "/cone_pipe/filtered_marker", 1)

        odom_sub = message_filters.Subscriber(self, Odometry, "/odometry/global")
        self.actualodom = message_filters.Cache(odom_sub, 1000) # needs to be the more than the max latency of perception in ms

        self.printmarkers: bool = True
        self.conesKDTree: KDNode = None
        self.bufferKDTree: KDNode = None

        # this is really the wrong way to do this but I just need a solution for now
        self.z_datum: float = 0
        self.z_readings: float = 1

        LOGGER.info("---Cone Pipeline Node Initalised---")
        self.logger.debug("---Cone Pipeline Node Initalised---")

    def odom_callback(self):#, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'ekf/map'
        t.child_frame_id = 'map'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = 0.0, 0.0, 0.3
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = 0.0, 0.0, 0.0, 1.0
        self._tf_publisher.sendTransform(t)


    def getNearestOdom(self, stamp):
        # need to switch this over to a position from a EKF with covariance
        locodom: Odometry = self.actualodom.getElemAfterTime(Time.from_msg(stamp))
        #cov = np.identity(6) * 0.0025 # standin covariance for ekf assuming the variance is sigma = 5cm with no covariance
        # if the nearest Odom in the cache is more than 0.05 sec off then just throw it away
        if locodom is not None:
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
        self.z_datum = (self.z_datum*(self.z_readings-1)+zHeight)/self.z_readings
        self.z_readings += 1

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

    def fusePoints(self, points, header: Header):
        # if we have cones in the cone tree than check if we can fuse our new points with them
        if self.conesKDTree is not None:
            for point in points:
                self.fuseCone(point)
        # if not we will check them against the buffer (or create a buffer) this should happen the tirst two cycles the fuse points is run (first there wont be either tree and the second time there wont be a cone tree yet)
        else:
            for point in points:
                self.bufferCone(point)
        
        header1 = Header()
        header1.stamp = header.stamp
        header1.frame_id = 'ekf/map'
        # if we have cones we are sure about than send them out in messages
        if self.conesKDTree is not None and self.conesKDTree.data is not None:
            # make a bool to determine if we are going to spend the time to generate markers
            msgs = self.printmarkers and self.filtered_markers.get_subscription_count() > 0
            # get all the cone elements from the tree structure
            curcones = self.conesKDTree.returnElements()
            # create the lists to fill with our elements
            conelist: List[QUTCone] = []
            conelist_cov: List[PointWithCovariance] = []
            markers: List[Marker] = []
            msgid = 0
            for cone in curcones:
                # should probabbly put a filter for how many times a cone has actually been spotted
                if cone.covMax(0.5):
                    # create out messages to be published with the final cones that we found
                    pubpt_cov = PointWithCovariance()
                    pubpt = QUTCone()
                    # create a point at the location of the cone
                    point = Point()
                    point.x, point.y, point.z = cone.global_x, cone.global_y, cone.global_z
                    # create the row major 3x3 cov matrix for the message elements
                    pubpt_cov.covariance = cone.global_cov.flatten()
                    # set those parts of the message
                    pubpt_cov.position = point
                    pubpt_cov.color = cone.color
                    pubpt.location = point
                    # set its color
                    pubpt.color = cone.color

                    # append those elements to the list of elements for the message
                    #if True:#cone.color < 4:
                    conelist.append(pubpt)
                    conelist_cov.append(pubpt_cov)

                    if msgs:
                        markers.append(cone.getCov(msgid, False, header1))
                        markers.append(cone.getMarker(msgid+1, header1))
                    msgid += 2
            if msgs:
                mkr = MarkerArray()
                mkr.markers = markers
                self.filtered_markers.publish(mkr)
            
            # create a ConeDetectionStamped message
            coneListPub = ConeDetectionStamped()
            coneListPub.cones = conelist
            # idk what makes sense for setting the header on this detection, this is probably wrong but idc atm
            coneListPub.header = header
            coneListPub.header.frame_id = 'ekf/map'
            # create a PointWithCovarianceStampedArray message
            coneListPubCov = PointWithCovarianceArrayStamped()
            coneListPubCov.points = conelist_cov
            coneListPubCov.header = header
            coneListPubCov.header.frame_id = 'ekf/map'
            # coneListPubCov.ns = ""
            # publish the messages
            self.filtered_cones_pub.publish(coneListPub)
            self.filtered_cones_pub_cov.publish(coneListPubCov)

        # need to make a section to remove old points from the buffer
        if self.bufferKDTree is not None and self.bufferKDTree.data is not None:
            for point in self.bufferKDTree.returnElements():
                if self.get_clock().now() - Duration(nanoseconds=2*10**9) > Time.from_msg(header.stamp):
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
                    elif (point.nMeasurments > 3 and point.covMin(0.1)) or point.global_z-self.z_datum < -0.3 or point.global_z-self.z_datum > 1.0: # 0.1 and 0.6 for sim
                        self.conesKDTree.remove(point)
                        self.conesKDTree.rebalance()
                if point.global_z-self.z_datum < -0.3 or point.global_z-self.z_datum > 1.0: # 0.1 and 0.6 for sim
                    print(f"X: {point.global_x}, Y: {point.global_y}, Z: {point.global_z}, Z_datum: {self.z_datum}, dZ: {point.global_z-self.z_datum}")
                    self.conesKDTree.remove(point)
                    self.conesKDTree.rebalance()

    def curTransform(self, header: Header):
        # create a list of points to be transformed
        to_frame_rel = self.target_frame
        from_frame_rel = header.frame_id
        # beacuse reasons for fetting the sim to work for now
        if from_frame_rel == 'map':
            from_frame_rel = 'ekf/map'
        try:
            now = Time.from_msg(header.stamp)
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now,
                Duration(nanoseconds=40000000))
            return trans
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return None

    def gen_headers(self, header: Header):
        # create a header for the cone messages
        header1 = Header()
        header1.stamp = header.stamp
        header1.frame_id = "fsds/FSCar"
        header2 = Header()
        header2.frame_id = 'ekf/map'
        header2.stamp = header.stamp
        return header1, header2

    def lidarCallback(self, points: PointWithCovarianceArrayStamped):
        if len(points.points) > 0:
            msgs = self.printmarkers and self.lidar_markers.get_subscription_count() > 0
            header = points.header
            header1, header2 = self.gen_headers(header)
            odomloc = self.getNearestOdom(header.stamp)
            
            if odomloc is None:
                return None

            curTransform = self.curTransform(header)
            
            z_datum = odomloc.pose.pose.position.z
            self.updateZDatum(z_datum)

            conelist: List[PointWithCov] = []
            markers: List[Marker] = []
            msgid = 0
            for point in points.points:
                tpnt = do_transform_point(point.position, curTransform)
                p = PointWithCov(tpnt.x, tpnt.y, tpnt.z, np.array(point.covariance).reshape((3,3)), 4)
                p.translate(odomloc)
                if tpnt.z < 0.45 and tpnt.z > 0.15:
                    conelist.append(p)
                    if msgs:
                        markers.append(p.getCov(msgid, True, header2))
                        markers.append(p.getMarker(msgid+1, header2))
                        msgid += 2
            if msgs:
                mkr = MarkerArray()
                mkr.markers = markers
                self.lidar_markers.publish(mkr)
            self.fusePoints(conelist, header)

    def visionCallback(self, points: PointWithCovarianceArrayStamped):
        if len(points.points) > 0:
            msgs = self.printmarkers and self.vision_markers.get_subscription_count() > 0
            header = points.header
            header1, header2 = self.gen_headers(header)
            odomloc= self.getNearestOdom(header.stamp)

            if odomloc is None:
                return None

            curTransform = self.curTransform(header)

            z_datum = odomloc.pose.pose.position.z
            self.updateZDatum(z_datum)
            
            conelist: List[PointWithCov] = []
            markers: List[Marker] = []
            msgid = 0
            for point in points.points:
                tpnt = do_transform_point(point.position, curTransform)
                p = PointWithCov(tpnt.x, tpnt.y, tpnt.z, np.array(point.covariance).reshape((3,3)), point.color)
                if p.xydist(0,0) < 15:
                    p.translate(odomloc)
                    conelist.append(p)
                    if msgs:
                        markers.append(p.getCov(msgid, True, header2))
                        markers.append(p.getMarker(msgid + 1, header2))
                        msgid += 2
            if msgs:
                mkr = MarkerArray()
                mkr.markers = markers
                self.vision_markers.publish(mkr)
            self.fusePoints(conelist, header)



def main(args=sys.argv[1:]):
    # defaults args
    loglevel = 'info'
    print_logs = False

    # processing args
    opts, arg = getopt.getopt(args, str(), ['log=', 'print_logs', 'length=', 'ros-args'])

    # TODO: provide documentation for different options
    for opt, arg in opts:
        if opt == '--log':
            loglevel = arg
        elif opt == '--print_logs':
            print_logs = True
        else:
            pass

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

