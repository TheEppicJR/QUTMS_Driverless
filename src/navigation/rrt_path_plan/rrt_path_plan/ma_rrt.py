"""
Path Planning Sample Code with RRT*

author: AtsushiSakai(@Atsushi_twi)
with edits of Maxim Yastremsky(@MaxMagazin)

"""

import random
import math
import copy
import numpy as np
from .kdtree import Node as kdNode, create
from .kdtree import KDNode
from .point import Point, PointWithCov, Edge, Triangle
from typing import List, Tuple


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y, yaw):
        self.x: float = x
        self.y: float = y
        self.yaw: float = yaw
        self.cost: float = 0.0
        self.parent = None

    def __str__(self):
        return str(round(self.x, 2)) + "," + str(round(self.y,2)) + "," + str(math.degrees(self.yaw)) + "," + str(self.cost)

    def __eq__(self, other: "Node"):
        return self.x == other.x and self.y == other.y and self.yaw == other.yaw and self.cost == other.cost

    def __repr__(self):
        return str(self)

    def __deepcopy__(self, memodict={}):
        copy_object = Node(self.x, self.y, self.yaw)
        copy_object.cost = self.cost
        copy_object.parent = self.parent
        return copy_object

    def __len__(self):
        return 2

    def __getitem__(self, i):
        return (self.x, self.y)[i]
    
    def to_tuple(self) -> Tuple:
        return (self.x, self.y)

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, planDistance, triangles: KDNode, points: KDNode, safedistance: float, expandDis=0.5, turnAngle=30, maxIter=1000, rrtTargets = None):

        self.start = Node(start[0], start[1], start[2])
        self.startYaw = start[2]
        self.nodeList: KDNode = create([self.start])

        self.planDistance = planDistance
        self.expandDis = expandDis
        self.turnAngle = math.radians(turnAngle)

        self.maxDepth = int(planDistance / expandDis)

        self.maxIter = maxIter
        self.triangles: KDNode = triangles
        self.points: KDNode = points
        self.safedistance: float = safedistance
        self.rrtTargets = rrtTargets

        self.aboveMaxDistance = 0
        self.belowMaxDistance = 0
        self.collisionHit = 0
        self.doubleNodeCount = 0

        self.savedRandoms = []

    def Planning(self):
        
        self.leafNodes = []

        for i in range(self.maxIter):
            rnd = self.get_random_point_from_target_list()

            nearestNodeRaw: kdNode = self.nodeList.search_nn_point(rnd[0], rnd[1])[0]
            if nearestNodeRaw is None or nearestNodeRaw.data is None:
                continue

            nearestNode: Node = nearestNodeRaw.data

            if (nearestNode.cost >= self.planDistance):
                continue

            newNode = self.steerConstrained(rnd, nearestNode)

            # due to angle constraints it is possible that similar node is generated
            if self.nodeList.search_nn_point(newNode.x, newNode.y)[1] < 0.05:
                continue

            if self.__CollisionCheck(newNode):
                self.nodeList.add(newNode)
                if not self.nodeList.is_balanced:
                    self.nodeList.rebalance()

                if (newNode.cost >= self.planDistance):
                    self.leafNodes.append(newNode)

            

        return self.nodeList, self.leafNodes

    def steerConstrained(self, rnd, nearestNode: Node):
        # expand tree
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

        # dynamic constraints
        angleChange = self.pi_2_pi(theta - nearestNode.yaw)

        angle30degree = math.radians(30)

        if angleChange > angle30degree:
            angleChange = self.turnAngle
        elif angleChange >= -angle30degree:
            angleChange = 0
        else:
            angleChange = -self.turnAngle

        newNode = copy.deepcopy(nearestNode)
        newNode.yaw += angleChange
        newNode.x += self.expandDis * math.cos(newNode.yaw)
        newNode.y += self.expandDis * math.sin(newNode.yaw)

        newNode.cost += self.expandDis
        newNode.parent = nearestNode.to_tuple()

        return newNode

    def pi_2_pi(self, angle: float):
        return (angle + math.pi) % (2*math.pi) - math.pi

    def get_random_point(self):

        randX = random.uniform(0, self.planDistance)
        randY = random.uniform(-self.planDistance, self.planDistance)
        rnd = [randX, randY]

        car_rot_mat = np.array([[math.cos(self.startYaw), -math.sin(self.startYaw)], [math.sin(self.startYaw), math.cos(self.startYaw)]])
        rotatedRnd = np.dot(car_rot_mat, rnd)

        rotatedRnd = [rotatedRnd[0] + self.start.x, rotatedRnd[1] + self.start.y]
        return rotatedRnd

    def get_random_point_from_target_list(self):

        maxTargetAroundDist = 3

        if not self.rrtTargets:
            return self.get_random_point()

        targetId = np.random.randint(len(self.rrtTargets))
        x, y, oSize = self.rrtTargets[targetId]

        # square idea
        # randX = random.uniform(-maxTargetAroundDist, maxTargetAroundDist)
        # randY = random.uniform(-maxTargetAroundDist, maxTargetAroundDist)
        # finalRnd = [x + randX, y + randY]

        # circle idea
        randAngle = random.uniform(0, 2 * math.pi)
        randDist = random.uniform(oSize, maxTargetAroundDist)
        finalRnd = [x + randDist * math.cos(randAngle), y + randDist * math.sin(randAngle)]

        return finalRnd

    def check_collision_extend(self, nearNode: Node, theta: float, d: float):

        tmpNode: Node = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode):
                return False

        return True

    def __CollisionCheck(self, node: Node):
        if len(self.points.search_nn_dist_point(node.x, node.y, self.safedistance)) > 0:
            return False
        # get the closest triangle to the midpoint of the line
        triangle: Triangle = self.triangles.search_nn_point(node.x, node.y)[0].data
        # make sure a nonetype dosent fuck us
        if triangle is not None:
            # for each edge of the near triangle see if it intersects a edge of the triangle
            # im not exhaustivly certain this is 100% mathmatically correct but i think it is a valid efficent way to check
            for edge in triangle.getEdges():
                if edge.intersect(Point(node.x, node.y), Point(node.parent[0], node.parent[1])) and edge.color < 4:
                    return False  # collision
        return True  # safe

