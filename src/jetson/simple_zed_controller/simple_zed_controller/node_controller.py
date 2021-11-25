import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

from .threshold import Threshold
from .hsv_cv import get_coloured_objects
from .rect import Rect, Point, draw_box

from typing import Optional


cv_bridge = CvBridge()


LEFT_THRESH = Threshold(  # yellow
    lower=[20, 170, 100],
    upper=[40, 255, 255],
)

RIGHT_THRESH = Threshold(  # blue
    lower=[120, 160, 50],
    upper=[130, 255, 255],
)

LEFT_DISP_COLOUR = (0, 255, 255)  # bgr - yellow
RIGHT_DISP_COLOUR = (255, 0, 0)  # bgr - blue
TARGET_DISP_COLOUR = (0, 0, 255)  # bgr - red


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")
        self.create_subscription(
            Image,
            "/zed2i/zed_node/left/image_rect_color",
            self.image_callback,
            10,
        )
        self.get_logger().info("Controller Node Initalised")

    def image_callback(self, msg):
        logger = self.get_logger()
        logger.info("Recieved image")
        frame: np.ndarray = cv_bridge.imgmsg_to_cv2(msg)
        hsv_frame: np.ndarray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        left_objects = get_coloured_objects(hsv_frame, [LEFT_THRESH])
        right_objects = get_coloured_objects(hsv_frame, [RIGHT_THRESH])

        # find "closest" (aka lowest) contours
        closest_left: Optional[Rect] = None
        closest_right: Optional[Rect] = None

        if len(left_objects) > 0:
            closest_left = max(left_objects, key=lambda r: r.bc.y)
    
        if len(right_objects) > 0:
            closest_right = max(right_objects, key=lambda r: r.bc.y)

        if closest_left is not None:
            draw_box(frame, box=closest_left, colour=LEFT_DISP_COLOUR)
        
        if closest_right is not None:
            draw_box(frame, box=closest_right, colour=RIGHT_DISP_COLOUR)
        
        if closest_left is not None and closest_right is not None:
            logger.info("Calculating Target")
            target = closest_left.bc + (closest_right.bc - closest_left.bc) / 2
            cv2.drawMarker(frame, (target.x, target.y), TARGET_DISP_COLOUR, cv2.MARKER_TILTED_CROSS)
            width, height, _ = frame.shape
            bottom_center = Point(width/2, height)
            cv2.line(frame, (bottom_center.x, bottom_center.y), (target.x, target.y), TARGET_DISP_COLOUR, thickness=2)

        cv2.imshow("frame", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()