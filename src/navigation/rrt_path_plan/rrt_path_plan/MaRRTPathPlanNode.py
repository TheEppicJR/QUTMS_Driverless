"""
RRT Path Planning with multiple remote goals.

author: Maxim Yastremsky(@MaxMagazin)
based on the work of AtsushiSakai(@Atsushi_twi)

"""
# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# import ROS2 message libraries
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point as PointMsg
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration as DurationMsg
from nav_msgs.msg import Odometry

import cProfile
import pstats

# import custom message libraries
from driverless_msgs.msg import Waypoint, WaypointsArray, PointWithCovarianceStamped, PointWithCovarianceStampedArray
from fs_msgs.msg import Track
from fs_msgs.msg import Cone as FSCone

from .ma_rrt import RRT
from .ma_rrt import Node as rrtNode

# import required sub modules
from .point import PointWithCov, Edge, Triangle, Point
from .kdtree import create, KDNode


# For odometry message
from transforms3d.euler import quat2euler

from scipy.spatial import Delaunay

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
from colour import Color
LOGGER = logging.getLogger(__name__)

red = Color("red")
blue = Color("blue")
col_range = list(blue.range_to(red, 100))


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def normalise(n, max):
    return clamp(int((n/max)*100), 1, 99)

class MaRRTPathPlanNode(Node):
    # All variables, placed here are static

    def __init__(self):
        super().__init__("rtt_planner")
        # Get all parameters from launch file
        self.shouldPublishWaypoints = True

        waypointsFrequency = 5
        self.waypointsPublishInterval = 1.0 / waypointsFrequency
        self.lastPublishWaypointsTime = 0

        self.logger = self.get_logger()

        self.create_subscription(PointWithCovarianceStampedArray, "/cone_pipe/cone_detection_cov", self.mapCallback, 10) 

        self.track_markers: Publisher = self.create_publisher(MarkerArray, "/cone_pipe/track_marker", 1)

        self.delaunayLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/delaunay_lines", 1)
        self.leftLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/left_line", 1)
        self.rightLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/right_line", 1)
        self.startLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/start_line", 1)
        self.delLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/del_line", 1)
        self.qsLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/qs_line", 1)
        self.clLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/cl_line", 1)

        self.create_subscription(Odometry, "/odometry/global", self.odometryCallback, 10) # "/testing_only/odom"


        # Create publishers
        self.waypointsPub: Publisher = self.create_publisher(WaypointsArray, "/waypoints", 1)
        self.trackPub: Publisher = self.create_publisher(Track, "/cone_pipe/track", 1)

        # visuals
        self.treeVisualPub: Publisher = self.create_publisher(MarkerArray, "/visual/tree_marker_array", 0)
        self.bestBranchVisualPub: Publisher = self.create_publisher(Marker, "/visual/best_tree_branch", 1)
        self.newWaypointsVisualPub: Publisher = self.create_publisher(Marker, "/visual/new_waypoints", 1)
        self.filteredBranchVisualPub: Publisher = self.create_publisher(Marker, "/visual/filtered_tree_branch", 1)
        self.waypointsVisualPub: Publisher = self.create_publisher(MarkerArray, "/visual/waypoints", 1)

        self.carPosX = 0.0
        self.carPosY = 0.0
        self.carPosYaw = 0.0

        self.map = []
        self.savedWaypoints = []
        self.preliminaryloopclosure = False
        self.loopclosure = False

        self.coneKDTree: KDNode = None
        self.curWaypoints: KDNode = None

        self.rrt = None

        self.filteredBestBranch = []
        self.discardAmount = 0

        self.printmarkers: bool = True

        LOGGER.info("---Cone Pipeline Node Initalised---")
        self.logger.debug("---Cone Pipeline Node Initalised---")

        print("MaRRTPathPlanNode Constructor has been called")

    def __del__(self):
        print('MaRRTPathPlanNode: Destructor called.')

    def mapCallback(self, track_msg: PointWithCovarianceStampedArray):
        self.map = track_msg.points
        quecones = self.getDelaunayEdges(self.map)
        #self.publishDelaunayEdgesVisual()
        if quecones:
            self.getEdges()

    def getEdges(self):
        leftHandEdges: List[Edge] = []
        rightHandEdges: List[Edge] = []
        discardedEdges: List[Edge] = []
        startingLineEdges: List[Edge] = []
        qsLineEdges: List[Edge] = []
        clLineEdges: List[Edge] = []
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
                        elif edge.color == 7: clLineEdges.append(edge)
                        else: discardedEdges.append(edge)
                        edge.calledFor = True
                        ne.calledFor = True
                    
        leftHandMsg = self.trackEdgesVisual(leftHandEdges, 0)
        rightHandMsg = self.trackEdgesVisual(rightHandEdges, 1)
        discardedEdgesMsg = self.trackEdgesVisual(discardedEdges, 5)
        startingEdgesMsg = self.trackEdgesVisual(startingLineEdges, 3)
        qsEdgesMsg = self.trackEdgesVisual(qsLineEdges, 2)
        clEdgesMsg = self.trackEdgesVisual(clLineEdges, 7)
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
        if clEdgesMsg is not None:
            self.clLinesVisualPub.publish(clEdgesMsg)
        if allEdgesMsg is not None:
            self.delaunayLinesVisualPub.publish(allEdgesMsg)


    def getDelaunayEdges(self, frontCones):
        if len(frontCones) < 2: # no sense to calculate delaunay
            return False
        

        conePoints = np.zeros((len(frontCones), 2))
        coneList: List[PointWithCov] = []
        triangleList: List[Triangle] = []
        edgeList: List[Edge] = []
        clEdgeList: List[Edge] = []

        if len(frontCones) < 3: # no sense to calculate delaunay
            conePoints = np.zeros((len(frontCones)+1, 2))
            conePoints[2] = ([0, 0])

        for i in range(len(frontCones)):
            cone: PointWithCovarianceStamped = frontCones[i]
            conePoints[i] = ([cone.position.x, cone.position.y])
            coneList.append(PointWithCov(0, 0, 0, None, cone.color, cone.header, cone.position.x, cone.position.y, cone.position.z, np.array(cone.covariance).reshape((3,3))))

        tri = Delaunay(conePoints)
        self.coneKDTree = create(coneList)
        
        for simp in tri.simplices:
            p1 = self.coneKDTree.search_knn(Point(conePoints[simp[0]][0], conePoints[simp[0]][1]), 1)[0][0].data
            p2 = self.coneKDTree.search_knn(Point(conePoints[simp[1]][0], conePoints[simp[1]][1]), 1)[0][0].data
            p3 = self.coneKDTree.search_knn(Point(conePoints[simp[2]][0], conePoints[simp[2]][1]), 1)[0][0].data
            edges = [Edge(p1, p2), Edge(p2, p3), Edge(p3, p1)]
            if self.curWaypoints is not None:
                for edge in edges:
                    # if there is a waypoint next to it then make its color 7 (because they are centerline points)
                    if len(self.curWaypoints.search_nn_dist_point(edge.x, edge.y, 1)) > 0:
                        edge.color = 7
                        clEdgeList.append(edge)
            edgeList.append(edges[0])
            edgeList.append(edges[1])
            edgeList.append(edges[2])
            triangleList.append(Triangle(p1, p2, p3, edges[0], edges[1], edges[2]))

        self.triangleList: List[Triangle] = triangleList
        self.edgeList: List[Edge] = edgeList
        self.edgeKDTree = create(edgeList)
        self.triangleKDTree = create(triangleList)
        if len(clEdgeList) > 4:
            self.makeTrack(clEdgeList)
        return True

    def makeTrack(self, clEdges: List[Edge]):
        rhPoints: List[PointWithCov] = []
        lhPoints: List[PointWithCov] = []
        for edge in clEdges:
            for triangle in self.triangleKDTree.search_knn_point(edge.x, edge.y, 2):
                if edge in triangle.data.getEdges():
                    for point in triangle.data.getPoints():
                        if point.color == 1:
                            if point not in rhPoints:
                                rhPoints.append(point)
                        elif point.color == 0:
                            if point not in lhPoints:
                                lhPoints.append(point)
        trackCones: List[FSCone] = []

        if len(rhPoints) < 3 or len(lhPoints) < 3:
            return

        rhConesKD = create(rhPoints)
        rhnnind = rhPoints.index(rhConesKD.search_nn_point(0,0)[0].data)
        rhs = rhPoints[rhnnind]
        rhstart = rhs
        rhOrderedList: List[PointWithCov] = []
        rhPoints.pop(rhnnind)
        while len(rhPoints) > 0:
            cpoint = rhs
            pointmsg = PointMsg()
            pointmsg.x = cpoint.global_x
            pointmsg.y = cpoint.global_y
            pointmsg.z = cpoint.global_z
            coneMsg = FSCone()
            coneMsg.location = pointmsg
            coneMsg.color = 1

            trackCones.append(coneMsg)
            rhOrderedList.append(rhs)
            # there is no reason i can explain but if you do 4 points and reverse it as opposed to just doing 3 points normally but it works
            for point in reversed(rhConesKD.search_knn(rhs, 4)):
                if point[0].data in rhPoints:
                    rhs = point[0].data
            
            if not rhs == cpoint:
                ind = rhPoints.index(rhs)
                rhPoints.pop(ind)
            else:
                if len(rhPoints) > 0:
                    rhs = rhPoints[0]
                    rhPoints.pop(0)
                else:
                    break
        if rhOrderedList[0].dist(rhOrderedList[-1]) < 10:
            cpoint = rhOrderedList[0]
            pointmsg = PointMsg()
            pointmsg.x = cpoint.global_x
            pointmsg.y = cpoint.global_y
            pointmsg.z = cpoint.global_z
            coneMsg = FSCone()
            coneMsg.location = pointmsg
            coneMsg.color = 1

            trackCones.append(coneMsg)


        lhConesKD = create(lhPoints)
        lhnnind = lhPoints.index(lhConesKD.search_nn_point(rhstart.x, rhstart.y)[0].data)
        lhs = lhPoints[lhnnind]
        lhstart = lhs
        lhOrderedList: List[PointWithCov] = []
        lhPoints.pop(lhnnind)
        while len(lhPoints) > 0:
            cpoint = lhs
            pointmsg = PointMsg()
            pointmsg.x = cpoint.global_x
            pointmsg.y = cpoint.global_y
            pointmsg.z = cpoint.global_z
            coneMsg = FSCone()
            coneMsg.location = pointmsg
            coneMsg.color = 0

            trackCones.append(coneMsg)
            lhOrderedList.append(lhs)
            if len(lhOrderedList) < 2:
                 for point in reversed(lhConesKD.search_knn(rhOrderedList[1], 4)):
                    if point[0].data in lhPoints:
                        lhs = point[0].data
            else:
                for point in reversed(lhConesKD.search_knn(lhs, 4)):
                    if point[0].data in lhPoints:
                        lhs = point[0].data
            if not lhs == cpoint:
                ind = lhPoints.index(lhs)
                lhPoints.pop(ind)
            else:
                if len(lhPoints) > 0:
                    lhs = lhPoints[0]
                    lhPoints.pop(0)
                else:
                    break

        if lhOrderedList[0].dist(lhOrderedList[-1]) < 10:
            cpoint = lhOrderedList[0]
            pointmsg = PointMsg()
            pointmsg.x = cpoint.global_x
            pointmsg.y = cpoint.global_y
            pointmsg.z = cpoint.global_z
            coneMsg = FSCone()
            coneMsg.location = pointmsg
            coneMsg.color = 0

            trackCones.append(coneMsg)

        trackMsg = Track()
        trackMsg.track = trackCones

        self.trackPub.publish(trackMsg)




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
        elif cc == 7:
            marker.color.g = 1.0
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

    def odometryCallback(self, odom_msg: Odometry):
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        (roll, pitch, yaw)  = quat2euler(orientation_list)

        self.carPosX = odom_msg.pose.pose.position.x
        self.carPosY = odom_msg.pose.pose.position.y
        self.carPosYaw = yaw

    def sampleTree(self):

        if self.loopclosure and len(self.savedWaypoints) > 0:
            self.publishWaypoints()
            return


        if self.coneKDTree is None or self.coneKDTree.data is None:
            print("no cone data")
            return


        starttime = time.time()
        frontConesDist = 12
        frontCones: List[PointWithCov] = self.getFrontConeObstacles(frontConesDist)

        coneObstacleSize = 1.1
        coneObstacleList = []
        rrtConeTargets = []
        coneTargetsDistRatio = 0.5
        for cone in frontCones:
            if cone.global_z < 0.5 and cone.global_z > 0.3:
                coneObstacleList.append((cone.global_x, cone.global_y, coneObstacleSize))

                coneDist = self.dist(self.carPosX, self.carPosY, cone.global_x, cone.global_y)

                if coneDist > frontConesDist * coneTargetsDistRatio:
                    rrtConeTargets.append((cone.global_x, cone.global_y, coneObstacleSize))

        # Set Initial parameters
        start = [self.carPosX, self.carPosY, self.carPosYaw]
        iterationNumber = 100
        planDistance = 12
        expandDistance = 1
        expandAngle = 15

        # rrt planning
        rrt = RRT(start, planDistance, self.triangleKDTree, self.coneKDTree, self.edgeKDTree, coneObstacleSize, expandDis=expandDistance, turnAngle=expandAngle, maxIter=iterationNumber, rrtTargets = rrtConeTargets)
        nodeList, leafNodes, failednodes = rrt.Planning()

        frontConesBiggerDist = 14
        largerGroupFrontCones: List[PointWithCov] = self.getFrontConeObstacles(frontConesBiggerDist)

        # BestBranch
        bestBranch = self.findBestBranch(leafNodes, nodeList, largerGroupFrontCones, coneObstacleSize, expandDistance, planDistance)

        self.publishTreeVisual(nodeList, leafNodes, failednodes)


        if bestBranch is not None:
            filteredBestBranch = self.getFilteredBestBranch(bestBranch)

            if filteredBestBranch:

                newWaypoints = []

                newWaypoints = self.getWaypointsFromEdges(filteredBestBranch)

                if newWaypoints:
                    self.mergeWaypoints(newWaypoints)

                self.publishWaypoints(newWaypoints)
        print(f"Took {time.time()-starttime} sec")


    def mergeWaypoints(self, newWaypoints):
        if not newWaypoints:
            return

        maxDistToSaveWaypoints = 2.0
        maxWaypointAmountToSave = 2
        waypointsDistTollerance = 0.5

        # check preliminary loopclosure
        if len(self.savedWaypoints) > 15:
            firstSavedWaypoint = self.savedWaypoints[0]

            for waypoint in reversed(newWaypoints):
                distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypoint[0], waypoint[1])
                if distDiff < waypointsDistTollerance:
                    self.preliminaryloopclosure = True
                    print("preliminaryloopclosure = True")
                    break


        newSavedPoints = []

        for i in range(len(newWaypoints)):
            waypointCandidate = newWaypoints[i]

            carWaypointDist = self.dist(self.carPosX, self.carPosY, waypointCandidate[0], waypointCandidate[1])

            if i >= maxWaypointAmountToSave or carWaypointDist > maxDistToSaveWaypoints:
                break
            else:
                for savedWaypoint in reversed(self.savedWaypoints):
                    waypointsDistDiff = self.dist(savedWaypoint[0], savedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                    if waypointsDistDiff < waypointsDistTollerance:
                        self.savedWaypoints.remove(savedWaypoint) #remove similar
                        break

                if (self.preliminaryloopclosure):
                    distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                    if distDiff < waypointsDistTollerance:
                        #self.loopclosure = True
                        print("loopclosure = True")
                        break

                self.savedWaypoints.append(waypointCandidate)
                newSavedPoints.append(waypointCandidate)

        if newSavedPoints: # make self.savedWaypoints and newWaypoints having no intersection
            for point in newSavedPoints:
                newWaypoints.remove(point)

        clPoints: List[Point] = []
        for waypoint in self.savedWaypoints:
            clPoints.append(Point(waypoint[0], waypoint[1]))

        if len(clPoints) < 1:
            self.curWaypoints = None
        else:
            self.curWaypoints = create(clPoints)


    def getWaypointsFromEdges(self, filteredBranch):

        waypoints = []
        for i in range (len(filteredBranch) - 1):
            node1 = filteredBranch[i]
            node2 = filteredBranch[i+1]
            a1 = np.array([node1.x, node1.y])
            a2 = np.array([node2.x, node2.y])

            maxAcceptedEdgeLength = 7
            maxEdgePartsRatio = 3

            intersectedEdges = []
            nearedges = self.edgeKDTree.search_nn_dist_point((node1.x + node2.x)/2, (node1.y  +node2.y)/2, 3)
            for edge in nearedges:
                # this can definitly be improved for speed i just have no idea what it is trying to accomplish

                b1 = np.array([edge.x1, edge.y1])
                b2 = np.array([edge.x2, edge.y2])

                if self.getLineSegmentIntersection(a1, a2, b1, b2):
                    if edge.length() < maxAcceptedEdgeLength:
                        edge.intersection = self.getLineIntersection(a1, a2, b1, b2)

                        edgePartsRatio = edge.getPartsLengthRatio()

                        if edgePartsRatio < maxEdgePartsRatio:
                            intersectedEdges.append(edge)

            if intersectedEdges:

                if len(intersectedEdges) == 1:
                    edge = intersectedEdges[0]

                    waypoints.append(edge.getMiddlePoint())
                else:
                    intersectedEdges.sort(key=lambda edge: self.dist(node1.x, node1.y, edge.intersection[0], edge.intersection[1], shouldSqrt = False))

                    for edge in intersectedEdges:
                        waypoints.append(edge.getMiddlePoint())

        return waypoints

    def dist(self, x1, y1, x2, y2, shouldSqrt = True):
        distSq = (x1 - x2) ** 2 + (y1 - y2) ** 2
        return math.sqrt(distSq) if shouldSqrt else distSq

    def publishWaypoints(self, newWaypoints = None):
        if (time.time() - self.lastPublishWaypointsTime) < self.waypointsPublishInterval:
            return

        waypointsArray = WaypointsArray()
        waypointsArray.header.frame_id = "map"

        waypointsArray.preliminaryloopclosure = self.preliminaryloopclosure
        waypointsArray.loopclosure = self.loopclosure

        for i in range(len(self.savedWaypoints)):
            waypoint = self.savedWaypoints[i]
            waypointId = len(waypointsArray.waypoints)
            w = Waypoint()
            w.id = float(waypointId)
            w.x = waypoint[0]
            w.y = waypoint[1]
            waypointsArray.waypoints.append(w)

        if newWaypoints is not None:
            for i in range(len(newWaypoints)):
                waypoint = newWaypoints[i]
                waypointId = len(waypointsArray.waypoints)
                w = Waypoint()
                w.id = float(waypointId)
                w.x = waypoint[0]
                w.y = waypoint[1]
                waypointsArray.waypoints.append(w)

        if self.shouldPublishWaypoints:
            self.waypointsPub.publish(waypointsArray)

            self.lastPublishWaypointsTime = time.time()

            self.publishWaypointsVisuals(newWaypoints)

    def publishWaypointsVisuals(self, newWaypoints = None):

        markerArray = MarkerArray()
        path_markers: List[PointMsg] = []
        path_markers2: List[PointMsg] = []
        markers: List[Marker] = []

        savedWaypointsMarker = Marker()
        savedWaypointsMarker.header.frame_id = "map"
        savedWaypointsMarker.lifetime = DurationMsg(sec=1)
        savedWaypointsMarker.ns = "saved-publishWaypointsVisuals"
        savedWaypointsMarker.id = 1

        savedWaypointsMarker.type = savedWaypointsMarker.SPHERE_LIST
        savedWaypointsMarker.action = savedWaypointsMarker.ADD
        savedWaypointsMarker.pose.orientation.w = 1.0
        savedWaypointsMarker.scale.x = 0.15
        savedWaypointsMarker.scale.y = 0.15
        savedWaypointsMarker.scale.z = 0.15

        savedWaypointsMarker.color.a = 1.0
        savedWaypointsMarker.color.b = 1.0

        for waypoint in self.savedWaypoints:
            p = PointMsg()
            p.x = waypoint[0]
            p.y = waypoint[1]
            p.z = 0.0
            path_markers.append(p)

        savedWaypointsMarker.points = path_markers

        markers.append(savedWaypointsMarker)

        if newWaypoints is not None:
            newWaypointsMarker = Marker()
            newWaypointsMarker.header.frame_id = "map"
            newWaypointsMarker.lifetime = DurationMsg(sec=1)
            newWaypointsMarker.ns = "new-publishWaypointsVisuals"
            newWaypointsMarker.id = 2

            newWaypointsMarker.type = newWaypointsMarker.SPHERE_LIST
            newWaypointsMarker.action = newWaypointsMarker.ADD
            newWaypointsMarker.pose.orientation.w = 1.0
            newWaypointsMarker.scale.x = 0.3
            newWaypointsMarker.scale.y = 0.3
            newWaypointsMarker.scale.z = 0.3

            newWaypointsMarker.color.a = 0.65
            newWaypointsMarker.color.b = 1.0

            for waypoint in newWaypoints:
                p = PointMsg()
                p.x = waypoint[0]
                p.y = waypoint[1]
                p.z = 0.0
                path_markers2.append(p)
            newWaypointsMarker.points = path_markers2
            self.newWaypointsVisualPub.publish(newWaypointsMarker)
            markers.append(newWaypointsMarker)
        
        markerArray.markers = markers
        self.waypointsVisualPub.publish(markerArray)

    def getLineIntersection(self, a1, a2, b1, b2):
        """
        Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
        a1: [x, y] a point on the first line
        a2: [x, y] another point on the first line
        b1: [x, y] a point on the second line
        b2: [x, y] another point on the second line
        https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
        """
        s = np.vstack([a1,a2,b1,b2])        # s for stacked
        h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
        l1 = np.cross(h[0], h[1])           # get first line
        l2 = np.cross(h[2], h[3])           # get second line
        x, y, z = np.cross(l1, l2)          # point of intersection
        if z == 0:                          # lines are parallel
            return (float('inf'), float('inf'))
        return (x/z, y/z)

    def getLineSegmentIntersection(self, a1, a2, b1, b2):
        # https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
        # Return true if line segments a1a2 and b1b2 intersect
        # return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
        return self.ccw(a1,b1,b2) != self.ccw(a2,b1,b2) and self.ccw(a1,a2,b1) != self.ccw(a1,a2,b2)

    def ccw(self, A, B, C):
        # if three points are listed in a counterclockwise order.
        # return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    def getFilteredBestBranch(self, bestBranch):
        if not bestBranch:
            return

        everyPointDistChangeLimit = 2.0
        newPointFilter = 0.2
        maxDiscardAmountForReset = 2

        if not self.filteredBestBranch:
            self.filteredBestBranch: List(rrtNode) = list(bestBranch)
        else:
            changeRate = 0
            shouldDiscard = False
            for i in range(len(bestBranch)-1):
                node: rrtNode = bestBranch[i]

                # sometimes this is empty and it throws a error but idk why
                #print(f"{i} {len(self.filteredBestBranch)}")
                filteredNode = self.filteredBestBranch[i]

                dist = math.sqrt((node.x - filteredNode.x) ** 2 + (node.y - filteredNode.y) ** 2)
                if dist > everyPointDistChangeLimit: # changed too much, skip this branch
                    shouldDiscard = True
                    self.discardAmount += 1

                    if self.discardAmount >= maxDiscardAmountForReset:
                        self.discardAmount = 0
                        self.filteredBestBranch = list(bestBranch)
                    break

                changeRate += (everyPointDistChangeLimit - dist)

            if not shouldDiscard:
                for i in range(len(bestBranch)-1):
                    self.filteredBestBranch[i].x = self.filteredBestBranch[i].x * (1 - newPointFilter) + newPointFilter * bestBranch[i].x
                    self.filteredBestBranch[i].y = self.filteredBestBranch[i].y * (1 - newPointFilter) + newPointFilter * bestBranch[i].y

                self.discardAmount = 0

        self.publishFilteredBranchVisual()
        return list(self.filteredBestBranch) # return copy

    def publishDelaunayEdgesVisual(self, edges):
        if not edges:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.lifetime = DurationMsg(sec=1)
        marker.ns = "publishDelaunayLinesVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05

        marker.pose.orientation.w = 1.0

        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.b = 1.0

        path_markers: List[PointMsg] = []

        for edge in edges:
            # print edge

            p1 = PointMsg()
            p1.x = edge.x1
            p1.y = edge.y1
            p1.z = 0.0
            p2 = PointMsg()
            p2.x = edge.x2
            p2.y = edge.y2
            p2.z = 0.0

            path_markers.append(p1)
            path_markers.append(p2)

        marker.points = path_markers

        self.delaunayLinesVisualPub.publish(marker)



    def findBestBranch(self, leafNodes: List[rrtNode], nodeList: KDNode, largerGroupFrontCones: List[PointWithCov], coneObstacleSize: float, expandDistance: float, planDistance: float):
        if not leafNodes:
            print("no leaf nodes")
            return None

        coneDistLimit = 4.0
        coneDistanceLimitSq = coneDistLimit * coneDistLimit

        bothSidesImproveFactor = 3
        minAcceptableBranchRating = 80 # fits good fsg18


        leafRatings = []
        for leaf in leafNodes:
            branchRating = 0
            node = leaf
            while node.parent is not None:
                nodeRating = 0

                leftCones: float = 0
                rightCones: float = 0
                if node.ratingcolor == 0:
                    for cone in largerGroupFrontCones:
                        coneDistSq = ((cone.global_x - node.x) ** 2 + (cone.global_y - node.y) ** 2)

                        if coneDistSq < coneDistanceLimitSq:
                            actualDist = math.sqrt(coneDistSq)

                            if actualDist < coneObstacleSize:
                                # node can be really close to a cone, cause we have new cones in this comparison, so skip these ones
                                continue

                            nodeRating += (coneDistLimit - actualDist)
                            
                            parentNodeRaw: KDNode = nodeList.search_nn_point(node.parent[0], node.parent[1])[0]
                            if parentNodeRaw is not None and parentNodeRaw.data is not None:
                                parentNode: rrtNode = parentNodeRaw.data
                                if self.isLeftCone(node, parentNode, cone):
                                    leftCones += 1
                                    if cone.color == 1:
                                        leftCones +=  0.5
                                        #nodeRating *= 1.05
                                        pass
                                else:
                                    rightCones += 1
                                    if cone.color == 0:
                                        rightCones += 0.5
                                        #nodeRating *= 1.05
                                        pass
                    dif = (1-abs((rightCones-leftCones)/(rightCones+leftCones+1)))

                    if ((leftCones == 0 and rightCones > 0) or (leftCones > 0 and rightCones == 0)):
                        nodeRating /= (bothSidesImproveFactor)# * dif)

                    if (leftCones > 0 and rightCones > 0):
                        nodeRating *= (bothSidesImproveFactor)# * dif)


                    # make conversion: (expandDistance to planDistance) -> (1 to 2)
                    nodeFactor = (node.cost - expandDistance)/(planDistance - expandDistance) + 1

                    node.ratingcolor = normalise(nodeRating * nodeFactor, 50)
                    node.rating = nodeRating * nodeFactor
                    
                    branchRating += nodeRating * nodeFactor
                else:
                    branchRating += node.rating
                # branchRating += nodeRating
                node: rrtNode = nodeList.search_nn_point(node.parent[0], node.parent[1])[0].data

            leafRatings.append(branchRating)

        maxRating = max(leafRatings)
        maxRatingInd = leafRatings.index(maxRating)

        node = leafNodes[maxRatingInd]

        if False:#maxRating < minAcceptableBranchRating:
            return None

        self.publishBestBranchVisual(nodeList, node)

        # it gets mad if i type these
        reverseBranch = []#List[rrtNode]
        reverseBranch.append(node)
        while node.parent is not None:
            node: rrtNode = nodeList.search_nn_point(node.parent[0], node.parent[1])[0].data
            reverseBranch.append(node)

        directBranch = []#List[rrtNode]
        for n in reversed(reverseBranch):
            directBranch.append(n)

        return directBranch

    def isLeftCone(self, node: rrtNode, parentNode: rrtNode, cone: PointWithCov):
        # //((b.X - a.X)*(cone.Y - a.Y) - (b.Y - a.Y)*(cone.X - a.X)) > 0;
        # if cone.color == 0:
        #     return True
        # if cone.color == 1:
        #     return False
        # THis determines if you were to follow the path between two nodes if the cone would end up on the left or right
        return ((node.x - parentNode.x) * (cone.global_y - parentNode.y) - (node.y - parentNode.y) * (cone.global_x - parentNode.x)) > 0

    def publishBestBranchVisual(self, nodeList, leafNode):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.lifetime = DurationMsg(nanosec=200000000)
        marker.ns = "publishBestBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1.0

        marker.color.a = 0.7
        marker.color.r = 1.0

        node = leafNode
        path_markers: List[PointMsg] = []

        parentNodeInd = node.parent
        while parentNodeInd is not None:
            parentNodeRaw = nodeList.search_nn_point(node.parent[0], node.parent[1])
            if parentNodeRaw is not None and parentNodeRaw[0].data is not None:
                parentNode: rrtNode = parentNodeRaw[0].data
                p = PointMsg()
                p.x = node.x
                p.y = node.y
                p.z = 0.0
                path_markers.append(p)

                p = PointMsg()
                p.x = parentNode.x
                p.y = parentNode.y
                p.z = 0.0
                path_markers.append(p)

                parentNodeInd = node.parent
                node = parentNode
                if node.parent is None:
                    parentNodeInd = None
            else:
                parentNodeInd = None

        marker.points = path_markers

        self.bestBranchVisualPub.publish(marker)

    def publishFilteredBranchVisual(self):

        if not self.filteredBestBranch:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.lifetime = DurationMsg(nanosec=200000000)
        marker.ns = "publisshFilteredBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1.0

        marker.color.a = 0.7
        marker.color.b = 1.0

        path_markers: List[PointMsg] = []

        for i in range(len(self.filteredBestBranch)):
            node = self.filteredBestBranch[i]
            p = PointMsg()
            p.x = node.x
            p.y = node.y
            p.z = 0.0
            if i != 0:
                path_markers.append(p)

            if i != len(self.filteredBestBranch) - 1:
                path_markers.append(p)

        marker.points = path_markers
        self.filteredBranchVisualPub.publish(marker)

    def publishTreeVisual(self, nodeList: KDNode, leafNodes, failedNodes):

        if not nodeList.returnElements() and not leafNodes:
            return

        markerArray = MarkerArray()

        # tree lines marker
        treeMarker = Marker()
        treeMarker.header.frame_id = "map"
        treeMarker.ns = "rrt"

        treeMarker.type = treeMarker.LINE_LIST
        treeMarker.action = treeMarker.ADD
        treeMarker.scale.x = 0.03

        treeMarker.pose.orientation.w = 1.0

        treeMarker.lifetime = DurationMsg(nanosec=2000000000)

        path_markers: List[PointMsg] = []
        path_markers2: List[PointMsg] = []
        path_markers3: List[PointMsg] = []
        path_colors: List[ColorRGBA] = []
        markers: List[Marker] = []

        for node in nodeList.returnElements():
            if node.parent is not None:
                p = PointMsg()
                p.x = node.x
                p.y = node.y
                p.z = 0.0
                path_markers.append(p)

                p = PointMsg()
                parentNode: rrtNode = nodeList.search_nn_point(node.parent[0], node.parent[1])[0].data
                p.x = parentNode.x
                p.y = parentNode.y
                p.z = 0.0
                path_markers.append(p)

                col = col_range[node.ratingcolor].get_rgb()
                line_colour = ColorRGBA()
                line_colour.a = 1.0 # alpha
                line_colour.r = col[0]
                line_colour.g = col[1]
                line_colour.b = col[2]
                path_colors.append(line_colour)
                col = col_range[parentNode.ratingcolor].get_rgb()
                line_colour = ColorRGBA()
                line_colour.a = 1.0 # alpha
                line_colour.r = col[0]
                line_colour.g = col[1]
                line_colour.b = col[2]
                path_colors.append(line_colour)

        treeMarker.points = path_markers
        treeMarker.colors = path_colors
        markers.append(treeMarker)

        # leaves nodes marker
        leavesMarker = Marker()
        leavesMarker.header.frame_id = "map"
        leavesMarker.lifetime = DurationMsg(nanosec=2000000000)
        leavesMarker.ns = "rrt-leaves"

        leavesMarker.type = leavesMarker.SPHERE_LIST
        leavesMarker.action = leavesMarker.ADD
        leavesMarker.pose.orientation.w = 1.0
        leavesMarker.scale.x = 0.15
        leavesMarker.scale.y = 0.15
        leavesMarker.scale.z = 0.15

        leavesMarker.color.a = 0.5
        leavesMarker.color.b = 0.1

        for node in leafNodes:
            p = PointMsg()
            p.x = node.x
            p.y = node.y
            p.z = 0.0
            path_markers2.append(p)

        leavesMarker.points = path_markers2
        markers.append(leavesMarker)

        # leaves nodes marker
        failedMarker = Marker()
        failedMarker.header.frame_id = "map"
        failedMarker.lifetime = DurationMsg(nanosec=2000000000)
        failedMarker.ns = "rrt-leaves"
        failedMarker.type = leavesMarker.SPHERE_LIST
        failedMarker.action = leavesMarker.ADD
        failedMarker.pose.orientation.w = 1.0
        failedMarker.scale.x = 0.15
        failedMarker.scale.y = 0.15
        failedMarker.scale.z = 0.15
        failedMarker.color.a = 0.5
        failedMarker.color.b = 0.0

        for node in failedNodes:
            p = PointMsg()
            p.x = node.x
            p.y = node.y
            p.z = 0.0
            path_markers3.append(p)

        failedMarker.points = path_markers3
        markers.append(failedMarker)

        markerArray.markers = markers
        # publis marker array
        self.treeVisualPub.publish(markerArray)

    def getFrontConeObstacles(self, frontDist):
        if self.coneKDTree is None or self.coneKDTree.data is None:
            return []

        headingVector = self.getHeadingVector()

        headingVectorOrt = [-headingVector[1], headingVector[0]]

        behindDist = 0.5
        carPosBehindPoint = [self.carPosX - behindDist * headingVector[0], self.carPosY - behindDist * headingVector[1]]


        frontDistSq = frontDist ** 2

        nearcones = self.coneKDTree.search_nn_dist_point(self.carPosX, self.carPosY, frontDist+0.75)

        frontConeList: List[PointWithCov] = []
        for cone in nearcones:
            if (headingVectorOrt[0] * (cone.global_y - carPosBehindPoint[1]) - headingVectorOrt[1] * (cone.global_x - carPosBehindPoint[0])) < 0:
                if ((cone.global_x - self.carPosX) ** 2 + (cone.global_y - self.carPosY) ** 2) < frontDistSq:
                    frontConeList.append(cone)
        return frontConeList

    def getHeadingVector(self):
        headingVector = [1.0, 0]
        carRotMat = np.array([[math.cos(self.carPosYaw), -math.sin(self.carPosYaw)], [math.sin(self.carPosYaw), math.cos(self.carPosYaw)]])
        headingVector = np.dot(carRotMat, headingVector)
        return headingVector

    def getConesInRadius(self, x, y, radius):
        coneList: List[PointWithCov] = []
        nearcones = self.coneKDTree.search_nn_dist_point(x, y, radius)
        radiusSq = radius * radius
        for conedat in nearcones:
            cone: PointWithCov = conedat[0].data
            if ((cone.global_x - x) ** 2 + (cone.global_y - y) ** 2) < radiusSq:
                coneList.append(cone)
        return coneList


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

    profiler = cProfile.Profile()
    profiler.enable()
    
    # begin ros node
    rclpy.init(args=args)

    node = MaRRTPathPlanNode()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(100)

    try:
        while rclpy.ok():
            node.sampleTree()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()

    rclpy.shutdown()
    thread.join()
    profiler.disable()
    stats = pstats.Stats(profiler).sort_stats(pstats.SortKey.TIME)
    stats.dump_stats(filename='needs_profiling.prof')


if __name__ == '__main__':
    main(sys.argv[1:])
