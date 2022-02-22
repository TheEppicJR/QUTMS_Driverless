# import ROS2 libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import message_filters
from ament_index_python.packages import get_package_share_directory
# import ROS2 message libraries
from sensor_msgs.msg import Image, CameraInfo
from rclpy.publisher import Publisher
from std_msgs.msg import Header
# import custom message libraries
from driverless_msgs.msg import Cone

# other python libraries
import os
import cv2
import numpy as np
from typing import List, Tuple

# import required sub modules
from .rect import Rect, draw_box
from .torch_inference import torch_init

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()

CAMERA_FOV = 110  # degrees


# display colour constants
Colour = Tuple[int, int, int]
YELLOW_DISP_COLOUR: Colour = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 165, 255)  # bgr - orange

# cone_colour, display_colour
YOLO_CONE_DETECTION_PARAMETERS = [
    (Cone.BLUE, BLUE_DISP_COLOUR),
    (Cone.YELLOW, YELLOW_DISP_COLOUR),
    (Cone.ORANGE_SMALL, ORANGE_DISP_COLOUR),
]

ConeMsgColour = int # define arbitrary variable type



class ZedNode(Node):
    def __init__(self):
        super().__init__("cone_annotator")

        self.create_subscription(Image, "/fsds/cam3", self.depth_callback, 10)
        self.create_subscription(Image, "/fsds/cam2", self.rgb_callback, 10)
        self.anno_img_publisher: Publisher = self.create_publisher(Image, "/fsds/cam3_real", 1)
        self.depth_img_publisher: Publisher = self.create_publisher(Image, "/zed2i/zed_node/depth/depth_registered", 1)
        self.rgb_img_publisher: Publisher = self.create_publisher(Image, "/zed2i/zed_node/rgb/image_rect_color", 1)
        self.rgb_img_info_publisher: Publisher = self.create_publisher(CameraInfo, "/zed2i/zed_node/rgb/camera_info", 1)
        self.camera_info = CameraInfo()
        self.camera_info.height = 1080
        self.camera_info.width = 1920
    

    def depth_callback(self, colour_msg: Image):#, colour_camera_info_msg: CameraInfo):
        self.depth_img_publisher.publish(colour_msg)
        # colour_frame: np.ndarray = cv_bridge.imgmsg_to_cv2(colour_msg, desired_encoding='32FC1')
        # backtorgb = cv2.cvtColor(colour_frame, cv2.COLOR_GRAY2RGB)
        # self.anno_img_publisher.publish(cv_bridge.cv2_to_imgmsg(backtorgb))
    
    def rgb_callback(self, colour_msg: Image):#, colour_camera_info_msg: CameraInfo):
        self.camera_info.header = colour_msg.header
        self.rgb_img_info_publisher.publish(self.camera_info)
        self.rgb_img_publisher.publish(colour_msg)



def main(args=None):
    rclpy.init(args=args)

    annotator_node = ZedNode()

    rclpy.spin(annotator_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
