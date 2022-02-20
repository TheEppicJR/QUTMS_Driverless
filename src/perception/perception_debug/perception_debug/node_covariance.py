# import ROS2 libraries
from cmath import sqrt
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.publisher import Publisher
from rclpy.clock import ClockType
from cv_bridge import CvBridge
import message_filters
from ament_index_python.packages import get_package_share_directory
# import ROS2 message libraries
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped
from fs_msgs.msg import ControlCommand, Track


# other python libraries
import os
from math import sin, cos, radians, isnan, isinf, atan2
import numpy as np
from typing import List, Tuple, Callable
import time
import enum
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
import cv2

cv_bridge = CvBridge()


def normalize_angle(angle: float) -> float:
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

# For odometry message
from transforms3d.euler import quat2euler

# need to add histogram for both so you can see what the density of data is (survivorship bias)
# this is really not eligant and could use tidying up

class CovNode(Node):
    def __init__(self):
        super().__init__("cone_cov")

        # create the critical subscriptions
        self.create_subscription(ConeDetectionStamped, "/detector/cone_detection", self.visionCallback, 10)
        self.create_subscription(ConeDetectionStamped, "lidar/cone_detection", self.lidarCallback, 10) # "/cone_sensing/cones"
        self.create_subscription(Track, "/testing_only/track", self.mapCallback, 10)
        sub = message_filters.Subscriber(self, Odometry, "/testing_only/odom")
        self.cache = message_filters.Cache(sub, 100)

        # create debug publishers
        self.lidar_dx_publisher: Publisher = self.create_publisher(Image, "/lidar_debug/dx", 1)
        self.lidar_dy_publisher: Publisher = self.create_publisher(Image, "/lidar_debug/dy", 1)
        self.lidar_dz_publisher: Publisher = self.create_publisher(Image, "/lidar_debug/dz", 1)
        self.lidar_da_publisher: Publisher = self.create_publisher(Image, "/lidar_debug/da", 1)
        self.lidar_cc_publisher: Publisher = self.create_publisher(Image, "/lidar_debug/cc", 1)
        self.lidar_ee_publisher: Publisher = self.create_publisher(Image, "/lidar_debug/ee", 1)
        self.vision_dx_publisher: Publisher = self.create_publisher(Image, "/vision_debug/dx", 1)
        self.vision_dy_publisher: Publisher = self.create_publisher(Image, "/vision_debug/dy", 1)
        self.vision_dz_publisher: Publisher = self.create_publisher(Image, "/vision_debug/dz", 1)
        self.vision_da_publisher: Publisher = self.create_publisher(Image, "/vision_debug/da", 1)
        self.vision_cc_publisher: Publisher = self.create_publisher(Image, "/vision_debug/cc", 1)
        self.vision_ee_publisher: Publisher = self.create_publisher(Image, "/vision_debug/ee", 1)

        self.radial = False
        self.camerafov = 55
        self.lidarfov = 90
        # make the global variables
        self.lidarcones: np.array = None
        self.visioncones: np.array = None
        self.lidarconese: np.array = None
        self.visionconese: np.array = None
        self.map = None

    def pltsparce(self, x, y, d, center=False, camera=True):

        if self.radial:
            if camera:
                xmin, xmax = self.camerafov*-1, self.camerafov
                ymax = 25
            else:
                xmin, xmax = self.lidarfov*-1, self.lidarfov
                ymax = 30
            ymin = 0
        else:
            ymin = 0
            ymax = 20
            xmin = -10
            xmax = 10

        counts, xbins, ybins = np.histogram2d(x, y, bins=(20, 30), range=[[ymin, ymax], [xmin, xmax]])
        sums, _, _ = np.histogram2d(x, y, weights=d, bins=(xbins, ybins))
        fig, ax = plt.subplots()

        #ax.plot(x, y, 'o', markersize=2, color='grey')
        #ax.tricontourf(x, y, d, levels=levels)
        # https://matplotlib.org/stable/tutorials/colors/colormaps.html
        
        colors = 'coolwarm'
        if center:
            colors = 'Reds'
        with np.errstate(divide='ignore', invalid='ignore'):  # suppress possible divide-by-zero warnings
            m3 = ax.pcolormesh(ybins, xbins, sums / counts, cmap=colors)
        plt.colorbar(m3, ax=ax)

        fig.canvas.draw()
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        img  = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        plt.close()
        # img is rgb, convert to opencv's default bgr
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        return img


    def plot(self):
        if self.lidarcones is not None:
            #print(f"Lidar: {np.cov(self.lidarcones[:, 4:7], rowvar=False)}")
            if self.lidar_dx_publisher.get_subscription_count() > 0:
                self.lidar_dx_publisher.publish(cv_bridge.cv2_to_imgmsg(self.pltsparce(self.lidarcones[:, 0], self.lidarcones[:, 1], self.lidarcones[:, 4], camera=False), encoding="bgr8"))
            if self.lidar_dy_publisher.get_subscription_count() > 0:
                self.lidar_dy_publisher.publish(cv_bridge.cv2_to_imgmsg(self.pltsparce(self.lidarcones[:, 0], self.lidarcones[:, 1], self.lidarcones[:, 5], camera=False), encoding="bgr8"))
            if self.lidar_dz_publisher.get_subscription_count() > 0:
                self.lidar_dz_publisher.publish(cv_bridge.cv2_to_imgmsg(self.pltsparce(self.lidarcones[:, 0], self.lidarcones[:, 1], self.lidarcones[:, 6], camera=False), encoding="bgr8"))
            if self.lidar_da_publisher.get_subscription_count() > 0:
                self.lidar_da_publisher.publish(cv_bridge.cv2_to_imgmsg(self.pltsparce(self.lidarcones[:, 0], self.lidarcones[:, 1], self.lidarcones[:, 7], center=True, camera=False), encoding="bgr8"))
            if self.lidar_cc_publisher.get_subscription_count() > 0:
                self.lidar_cc_publisher.publish(cv_bridge.cv2_to_imgmsg(self.pltsparce(self.lidarcones[:, 0], self.lidarcones[:, 1], self.lidarcones[:, 9], camera=False), encoding="bgr8"))
        if self.lidarconese is not None:
            if self.lidar_ee_publisher.get_subscription_count() > 0:
                self.lidar_ee_publisher.publish(cv_bridge.cv2_to_imgmsg(self.pltsparce(self.lidarconese[:, 0], self.lidarconese[:, 1], self.lidarconese[:, 7], camera=False), encoding="bgr8"))
            
        if self.visioncones is not None:
            #print(f"Vision: {np.cov(self.visioncones[:, 4:7], rowvar=False)}")
            if self.vision_dx_publisher.get_subscription_count() > 0:
                self.vision_dx_publisher.publish(cv_bridge.cv2_to_imgmsg(self.pltsparce(self.visioncones[:, 0], self.visioncones[:, 1], self.visioncones[:, 4]), encoding="bgr8"))
            if self.vision_dy_publisher.get_subscription_count() > 0:
                self.vision_dy_publisher.publish(cv_bridge.cv2_to_imgmsg(self.pltsparce(self.visioncones[:, 0], self.visioncones[:, 1], self.visioncones[:, 5]), encoding="bgr8"))
            if self.lidar_dz_publisher.get_subscription_count() > 0:
                self.vision_dz_publisher.publish(cv_bridge.cv2_to_imgmsg(self.pltsparce(self.visioncones[:, 0], self.visioncones[:, 1], self.visioncones[:, 6]), encoding="bgr8"))
            if self.vision_da_publisher.get_subscription_count() > 0:
                self.vision_da_publisher.publish(cv_bridge.cv2_to_imgmsg(self.pltsparce(self.visioncones[:, 0], self.visioncones[:, 1], self.visioncones[:, 7], center=True), encoding="bgr8"))
            if self.vision_cc_publisher.get_subscription_count() > 0:
                self.vision_cc_publisher.publish(cv_bridge.cv2_to_imgmsg(self.pltsparce(self.visioncones[:, 0], self.visioncones[:, 1], self.visioncones[:, 9]), encoding="bgr8"))
        if self.visionconese is not None:
            if self.vision_ee_publisher.get_subscription_count() > 0:
                self.vision_ee_publisher.publish(cv_bridge.cv2_to_imgmsg(self.pltsparce(self.visionconese[:, 0], self.visionconese[:, 1], self.visionconese[:, 7]), encoding="bgr8"))


    def mapCallback(self, track_msg: Track):
        rawmap = track_msg.track
        cones = []
        self.conearr = []
        for cone in rawmap:
            cones.append([cone.location.x, cone.location.y, cone.location.z])
            self.conearr.append(cone)
        if len(cones) > 0:
            self.map = KDTree(np.array(cones))


    def visionCallback(self, cones: ConeDetectionStamped):
        delts, err = self.getDistances(cones.cones, cones.header)
        if delts is not None:
            if self.visioncones is not None:
                self.visioncones = np.concatenate((self.visioncones, delts), axis=0)
            else:
                self.visioncones = delts
        if err is not None:
            if self.visionconese is not None:
                self.visionconese = np.concatenate((self.visionconese, err), axis=0)
            else:
                self.visionconese = err
        self.plot()

    
    def lidarCallback(self, cones: ConeDetectionStamped):
        delts, err = self.getDistances(cones.cones, cones.header)
        if delts is not None:
            if self.lidarcones is not None:
                self.lidarcones = np.concatenate((self.lidarcones, delts), axis=0)
            else:
                self.lidarcones = delts
        if err is not None:
            if self.lidarconese is not None:
                self.lidarconese = np.concatenate((self.lidarconese, err), axis=0)
            else:
                self.lidarconese = err
        self.plot()


    def getDistances(self, cones: List, header: Header):
        # have to do the time stuff this way because the compating of time in the message_filters __init__.py is wack
        pos = self.cache.getElemBeforeTime(Time(seconds=header.stamp.sec, nanoseconds=header.stamp.nanosec, clock_type=ClockType.ROS_TIME))
        if pos and self.map is not None:
            orientation_q = pos.pose.pose.orientation
            orientation_list = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
            (roll, pitch, yaw)  = quat2euler(orientation_list)

            carPosX = pos.pose.pose.position.x
            carPosY = pos.pose.pose.position.y
            carPosZ = pos.pose.pose.position.z
            carPosYaw = yaw
            s = sin(carPosYaw)
            c = cos(carPosYaw)
            ns = sin(-carPosYaw)
            nc = cos(-carPosYaw)
            
            conePos = []
            errantPos = []
            for cone in cones:
                lx, ly, lz, pc = cone.location.x, cone.location.y, cone.location.z, cone.color
                # find where the points actually are in the global refrence frame
                gx, gy, gz = carPosX+lx*c-ly*s, carPosY+ly*c+lx*s, carPosZ+lz
                # get the closest cone
                dd, ii = self.map.query([[gx, gy, gz]], k=1)
                closestPoint = self.conearr[ii[0][0]]
                # put that cone back in the local regrence frame
                gax, gay, gaz, ac = closestPoint.location.x - carPosX, closestPoint.location.y - carPosY, closestPoint.location.z - carPosZ, closestPoint.color
                lax, lay, laz = gax*nc-gay*ns, gay*nc+gax*ns, gaz
                dx, dy, dz = lax-lx, lay-ly, laz-lz

                # have to take the real number part otherwise it breaks all of the plotting
                de = sqrt(dx*dx+dy*dy).real #+dz*dz
                cc = 1
                if pc == ac:
                    cc = 0

                if self.radial:
                    theta = atan2(ly, lx)
                    lx = sqrt(lx*lx+ly*ly).real
                    ly = normalize_angle(theta)*180/3.1415

                # now add that data to a array
                if de > 2:
                    errantPos.append([lx, ly, lz, pc, dx, dy, dz, de, ac, cc])
                else:
                    conePos.append([lx, ly, lz, pc, dx, dy, dz, de, ac, cc])
            if len(conePos) == 0:
                retP = None
            else:
                retP = np.array(conePos)
            if len(errantPos) == 0:
                retR = None
            else:
                retR = np.array(errantPos)
            return retP, retR
        return None, None

            


def main(args=None):
    rclpy.init(args=args)

    cov_node = CovNode()

    rclpy.spin(cov_node)
    rclpy.shutdown()


