"""
RRT Path Planning with multiple remote goals.

author: Maxim Yastremsky(@MaxMagazin)
based on the work of AtsushiSakai(@Atsushi_twi)

"""
# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from cv_bridge import CvBridge
import message_filters
# import ROS2 message libraries
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry

import cProfile
import pstats

# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped, Waypoint, WaypointsArray, PointWithCovarianceStamped, PointWithCovarianceStampedArray
from fs_msgs.msg import ControlCommand, Track

from .ma_rrt import RRT

# import required sub modules
from .point import PointWithCov
from . import kdtree


# For odometry message
from transforms3d.euler import quat2euler

from scipy.spatial import Delaunay

# Matrix/Array library
import numpy as np
# other python modules
import math
from math import sin, cos
from typing import List
import sys
import os
import getopt
import logging
import datetime
import pathlib
import threading
import time

LOGGER = logging.getLogger(__name__)


class MaRRTPathPlanNode(Node):
    # All variables, placed here are static

    def __init__(self):
        super().__init__("rtt_planner")
        # Get all parameters from launch file
        self.shouldPublishWaypoints = True

        waypointsFrequency = 5
        self.waypointsPublishInterval = 1.0 / waypointsFrequency
        self.lastPublishWaypointsTime = 0

        # All Subs and pubs
        self.create_subscription(PointWithCovarianceStampedArray, "/cone_pipe/cone_detection_cov", self.conesCallback, 10)
        self.create_subscription(Odometry, "/testing_only/odom", self.odometryCallback, 10)

        # Create publishers
        self.waypointsPub: Publisher = self.create_publisher(WaypointsArray, "/waypoints", 1)

        # visuals
        self.treeVisualPub: Publisher = self.create_publisher(MarkerArray, "/visual/tree_marker_array", 0)
        self.bestBranchVisualPub: Publisher = self.create_publisher(Marker, "/visual/best_tree_branch", 1)
        self.newWaypointsVisualPub: Publisher = self.create_publisher(Marker, "/visual/new_waypoints", 1)
        self.filteredBranchVisualPub: Publisher = self.create_publisher(Marker, "/visual/filtered_tree_branch", 1)
        self.delaunayLinesVisualPub: Publisher = self.create_publisher(Marker, "/visual/delaunay_lines", 1)
        self.waypointsVisualPub: Publisher = self.create_publisher(MarkerArray, "/visual/waypoints", 1)

        self.carPosX = 0.0
        self.carPosY = 0.0
        self.carPosYaw = 0.0

        self.map = []
        self.savedWaypoints = []
        self.preliminaryloopclosure = False
        self.loopclosure = False
        self.trackEdges: List[Edge] = []
        self.conesKDTree: KDNode = None

        self.rrt = None

        self.filteredBestBranch = []
        self.discardAmount = 0

        # print("MaRRTPathPlanNode Constructor has been called")

    def __del__(self):
        print('MaRRTPathPlanNode: Destructor called.')

    def odometryCallback(self, odom_msg: Odometry):
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        (roll, pitch, yaw)  = quat2euler(orientation_list)

        self.carPosX = odom_msg.pose.pose.position.x
        self.carPosY = odom_msg.pose.pose.position.y
        self.carPosYaw = yaw


    def conesCallback(self, cones):
        coneList: List[PointWithCov] = []

        for i in range(len(cones.points)):
            cone = cones.points[i]
            coneList.append(PointWithCov(0, 0, 0, None, cone.color, cone.header, cone.position.x, cone.position.y, cone.position.z, np.array(cone.covariance).reshape((3,3))))
        self.coneKDTree = kdtree.create(coneList)
        self.map = cones.points

    def sampleTree(self):

        if self.loopclosure and len(self.savedWaypoints) > 0:
            self.publishWaypoints()
            return


        if not self.map:
            return


        frontConesDist = 12
        frontCones = self.getFrontConeObstacles(self.map, frontConesDist)

        coneObstacleSize = 1.1
        coneObstacleList = []
        rrtConeTargets = []
        coneTargetsDistRatio = 0.5
        for cone in frontCones:
            coneObstacleList.append((cone.position.x, cone.position.y, coneObstacleSize))

            coneDist = self.dist(self.carPosX, self.carPosY, cone.position.x, cone.position.y)

            if coneDist > frontConesDist * coneTargetsDistRatio:
                rrtConeTargets.append((cone.position.x, cone.position.y, coneObstacleSize))

        # Set Initial parameters
        start = [self.carPosX, self.carPosY, self.carPosYaw]
        iterationNumber = 1000
        planDistance = 12
        expandDistance = 1
        expandAngle = 20

        # rrt planning
        rrt = RRT(start, planDistance, obstacleList=coneObstacleList, expandDis=expandDistance, turnAngle=expandAngle, maxIter=iterationNumber, rrtTargets = rrtConeTargets)
        nodeList, leafNodes = rrt.Planning()

        self.publishTreeVisual(nodeList, leafNodes)

        frontConesBiggerDist = 12
        largerGroupFrontCones = self.getFrontConeObstacles(self.map, frontConesBiggerDist)

        # BestBranch
        bestBranch = self.findBestBranch(leafNodes, nodeList, largerGroupFrontCones, coneObstacleSize, expandDistance, planDistance)


        if bestBranch:
            filteredBestBranch = self.getFilteredBestBranch(bestBranch)

            if filteredBestBranch:
                # Delaunay
                delaunayEdges = self.getDelaunayEdges(frontCones)

                self.publishDelaunayEdgesVisual(delaunayEdges)


                newWaypoints = []

                if delaunayEdges:

                    newWaypoints = self.getWaypointsFromEdges(filteredBestBranch, delaunayEdges)

                if newWaypoints:
                    self.mergeWaypoints(newWaypoints)

                self.publishWaypoints(newWaypoints)


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

    def getWaypointsFromEdges(self, filteredBranch, delaunayEdges):
        if not delaunayEdges:
            return

        waypoints = []
        for i in range (len(filteredBranch) - 1):
            node1 = filteredBranch[i]
            node2 = filteredBranch[i+1]
            a1 = np.array([node1.x, node1.y])
            a2 = np.array([node2.x, node2.y])

            maxAcceptedEdgeLength = 7
            maxEdgePartsRatio = 3

            intersectedEdges = []
            for edge in delaunayEdges:

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

    def getDelaunayEdges(self, frontCones):
        if len(frontCones) < 4: # no sense to calculate delaunay
            return

        conePoints = np.zeros((len(frontCones), 2))

        for i in range(len(frontCones)):
            cone = frontCones[i]
            conePoints[i] = ([cone.position.x, cone.position.y])

        tri = Delaunay(conePoints)

        delaunayEdges = []
        for simp in tri.simplices:

            for i in range(3):
                j = i + 1
                if j == 3:
                    j = 0
                edge = Edge(conePoints[simp[i]][0], conePoints[simp[i]][1], conePoints[simp[j]][0], conePoints[simp[j]][1])

                if edge not in delaunayEdges:
                    delaunayEdges.append(edge)

        return delaunayEdges

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
        path_markers: List[Point] = []
        path_markers2: List[Point] = []
        markers: List[Marker] = []

        savedWaypointsMarker = Marker()
        savedWaypointsMarker.header.frame_id = "map"
        savedWaypointsMarker.lifetime = Duration(sec=1)
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
            p = Point()
            p.x = waypoint[0]
            p.y = waypoint[1]
            p.z = 0.0
            path_markers.append(p)

        savedWaypointsMarker.points = path_markers

        markers.append(savedWaypointsMarker)

        if newWaypoints is not None:
            newWaypointsMarker = Marker()
            newWaypointsMarker.header.frame_id = "map"
            newWaypointsMarker.lifetime = Duration(sec=1)
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
                p = Point()
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
            self.filteredBestBranch = list(bestBranch)
        else:
            changeRate = 0
            shouldDiscard = False
            for i in range(len(bestBranch)):
                node = bestBranch[i]
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
            #     return
            # else:
                for i in range(len(bestBranch)):
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
        marker.lifetime = Duration(sec=1)
        marker.ns = "publishDelaunayLinesVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05

        marker.pose.orientation.w = 1.0

        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.b = 1.0

        path_markers: List[Point] = []

        for edge in edges:
            # print edge

            p1 = Point()
            p1.x = edge.x1
            p1.y = edge.y1
            p1.z = 0.0
            p2 = Point()
            p2.x = edge.x2
            p2.y = edge.y2
            p2.z = 0.0

            path_markers.append(p1)
            path_markers.append(p2)

        marker.points = path_markers

        self.delaunayLinesVisualPub.publish(marker)

    def isSideTrack(self, cone):
        if self.conesKDTree is not None and self.conesKDTree.data is not None:
            pt = PointWithCov(0, 0, 0, None, 4, Header(), cone.position.x, cone.position.y, 0.0, None)
            knn = self.coneKDTree.search_knn(pt, 1)
            if len(knn) > 0:
                if knn[0][0].dist(pt) < 1:
                    if knn[0][0].color == 0:
                        return True, True
                    elif knn[0][0].color == 0:
                        return True, False
        return False, False


    def findBestBranch(self, leafNodes, nodeList, largerGroupFrontCones, coneObstacleSize, expandDistance, planDistance):
        if not leafNodes:
            return

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

                leftCones = []
                rightCones = []

                for cone in largerGroupFrontCones:
                    coneDistSq = ((cone.position.x - node.x) ** 2 + (cone.position.y - node.y) ** 2)

                    if coneDistSq < coneDistanceLimitSq:
                        actualDist = math.sqrt(coneDistSq)

                        if actualDist < coneObstacleSize:
                            # node can be really close to a cone, cause we have new cones in this comparison, so skip these ones
                            continue

                        nodeRating += (coneDistLimit - actualDist)
                        # this is the wrong way to structure this to be efficent
                        side, isL = self.isSideTrack(cone)
                        if side:
                            if isL:
                                leftCones.append(cone)
                            else:
                                rightCones.append(cone)
                        else:
                            if self.isLeftCone(node, nodeList[node.parent], cone):
                                leftCones.append(cone)
                            else:
                                rightCones.append(cone)

                if ((len(leftCones) == 0 and len(rightCones)) > 0 or (len(leftCones) > 0 and len(rightCones) == 0)):
                    nodeRating /= bothSidesImproveFactor

                if (len(leftCones) > 0 and len(rightCones) > 0):
                    nodeRating *= bothSidesImproveFactor


                # make conversion: (expandDistance to planDistance) -> (1 to 2)
                nodeFactor = (node.cost - expandDistance)/(planDistance - expandDistance) + 1

                branchRating += nodeRating * nodeFactor
                # branchRating += nodeRating
                node = nodeList[node.parent]

            leafRatings.append(branchRating)

        maxRating = max(leafRatings)
        maxRatingInd = leafRatings.index(maxRating)

        node = leafNodes[maxRatingInd]

        if maxRating < minAcceptableBranchRating:
            return

        self.publishBestBranchVisual(nodeList, node)

        reverseBranch = []
        reverseBranch.append(node)
        while node.parent is not None:
            node = nodeList[node.parent]
            reverseBranch.append(node)

        directBranch = []
        for n in reversed(reverseBranch):
            directBranch.append(n)

        return directBranch

    def isLeftCone(self, node, parentNode, cone):
        # //((b.X - a.X)*(cone.Y - a.Y) - (b.Y - a.Y)*(cone.X - a.X)) > 0;
        return ((node.x - parentNode.x) * (cone.position.y - parentNode.y) - (node.y - parentNode.y) * (cone.position.x - parentNode.x)) > 0

    def publishBestBranchVisual(self, nodeList, leafNode):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.lifetime = Duration(nanosec=200000000)
        marker.ns = "publishBestBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1.0

        marker.color.a = 0.7
        marker.color.r = 1.0

        node = leafNode
        path_markers: List[Point] = []

        parentNodeInd = node.parent
        while parentNodeInd is not None:
            parentNode = nodeList[parentNodeInd]
            p = Point()
            p.x = node.x
            p.y = node.y
            p.z = 0.0
            path_markers.append(p)

            p = Point()
            p.x = parentNode.x
            p.y = parentNode.y
            p.z = 0.0
            path_markers.append(p)

            parentNodeInd = node.parent
            node = parentNode
        marker.points = path_markers

        self.bestBranchVisualPub.publish(marker)

    def publishFilteredBranchVisual(self):

        if not self.filteredBestBranch:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.lifetime = Duration(nanosec=200000000)
        marker.ns = "publisshFilteredBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1.0

        marker.color.a = 0.7
        marker.color.b = 1.0

        path_markers: List[Point] = []

        for i in range(len(self.filteredBestBranch)):
            node = self.filteredBestBranch[i]
            p = Point()
            p.x = node.x
            p.y = node.y
            p.z = 0.0
            if i != 0:
                path_markers.append(p)

            if i != len(self.filteredBestBranch) - 1:
                path_markers.append(p)

        marker.points = path_markers
        self.filteredBranchVisualPub.publish(marker)

    def publishTreeVisual(self, nodeList, leafNodes):

        if not nodeList and not leafNodes:
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

        treeMarker.color.a = 0.7
        treeMarker.color.g = 0.7

        treeMarker.lifetime = Duration(nanosec=200000000)

        path_markers: List[Point] = []
        path_markers2: List[Point] = []
        markers: List[Marker] = []

        for node in nodeList:
            if node.parent is not None:
                p = Point()
                p.x = node.x
                p.y = node.y
                p.z = 0.0
                path_markers.append(p)

                p = Point()
                p.x = nodeList[node.parent].x
                p.y = nodeList[node.parent].y
                p.z = 0.0
                path_markers.append(p)

        treeMarker.points = path_markers
        markers.append(treeMarker)

        # leaves nodes marker
        leavesMarker = Marker()
        leavesMarker.header.frame_id = "map"
        leavesMarker.lifetime = Duration(nanosec=200000000)
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
            p = Point()
            p.x = node.x
            p.y = node.y
            p.z = 0.0
            path_markers2.append(p)

        markers.append(leavesMarker)

        markerArray.markers = markers
        # publis marker array
        self.treeVisualPub.publish(markerArray)

    def getFrontConeObstacles(self, map, frontDist):
        if not map:
            return []

        headingVector = self.getHeadingVector()

        headingVectorOrt = [-headingVector[1], headingVector[0]]

        behindDist = 0.5
        carPosBehindPoint = [self.carPosX - behindDist * headingVector[0], self.carPosY - behindDist * headingVector[1]]


        frontDistSq = frontDist ** 2

        frontConeList = []
        for cone in map:
            if (headingVectorOrt[0] * (cone.position.y - carPosBehindPoint[1]) - headingVectorOrt[1] * (cone.position.x - carPosBehindPoint[0])) < 0:
                if ((cone.position.x - self.carPosX) ** 2 + (cone.position.y - self.carPosY) ** 2) < frontDistSq:
                    frontConeList.append(cone)
        return frontConeList

    def getHeadingVector(self):
        headingVector = [1.0, 0]
        carRotMat = np.array([[math.cos(self.carPosYaw), -math.sin(self.carPosYaw)], [math.sin(self.carPosYaw), math.cos(self.carPosYaw)]])
        headingVector = np.dot(carRotMat, headingVector)
        return headingVector

    def getConesInRadius(self, map, x, y, radius):
        coneList = []
        radiusSq = radius * radius
        for cone in map:
            if ((cone.position.x - x) ** 2 + (cone.position.y - y) ** 2) < radiusSq:
                coneList.append(cone)
        return coneList

class Edge():
    def __init__(self, x1, y1, x2, y2, color: int = 4):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.color = color
        self.intersection = None

    def getMiddlePoint(self):
        return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

    def length(self):
        return math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

    def getPartsLengthRatio(self):

        part1Length = math.sqrt((self.x1 - self.intersection[0]) ** 2 + (self.y1 - self.intersection[1]) ** 2)
        part2Length = math.sqrt((self.intersection[0] - self.x2) ** 2 + (self.intersection[1] - self.y2) ** 2)

        return max(part1Length, part2Length) / min(part1Length, part2Length)

    def __eq__(self, other):
        return (self.x1 == other.x1 and self.y1 == other.y1 and self.x2 == other.x2 and self.y2 == other.y2
             or self.x1 == other.x2 and self.y1 == other.y2 and self.x2 == other.x1 and self.y2 == other.y1)

    def __str__(self):
        return "(" + str(round(self.x1, 2)) + "," + str(round(self.y1,2)) + "),(" + str(round(self.x2, 2)) + "," + str(round(self.y2,2)) + ")"

    def __repr__(self):
        return str(self)



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

    # profiler = cProfile.Profile()
    # profiler.enable()
    
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
    # profiler.disable()
    # stats = pstats.Stats(profiler).sort_stats(pstats.SortKey.TIME)
    # stats.dump_stats(filename='needs_profiling.prof')


if __name__ == '__main__':
    main(sys.argv[1:])
