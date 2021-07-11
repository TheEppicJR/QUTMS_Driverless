import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TwistStamped

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

from .sub_module.simple_lidar import find_points
import math

class LidarProcessing(Node):
    def __init__(self):
        super().__init__('lidar_processing')
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            '/fsds/lidar/Lidar2',
            self.lidar_callback,
            10)
        self.lidar_subscription  # prevent unused variable warning

        self.math_publisher = self.create_publisher(
            Float32MultiArray, # will change if we return all cones back
            'math_output', 
            10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.avg_y = 0.0

        self.geo_subscription = self.create_subscription(
            TwistStamped,
            '/fsds/gss',
            self.geo_callback,
            10)
        self.geo_subscription 
        self.calc_throttle = 0.0


    def find_avg(self, cones):
        if len(cones) != 0:
            average_y = 0
            for cone in cones:
                average_y += cone[1]
            average_y = average_y / len(cones)

            return average_y
        
        else:
            return 0 


    def lidar_callback(self, pcl_msg):
        """ In here, we will call calculations to ideally get the 
        distance, angle, and reflectivity of the cones"""
        cones_range_cutoff = 7 # m      can tweak
        distance_cutoff = 0.1 # m       can tweak

        self.cones = find_points(pcl_msg, cones_range_cutoff, distance_cutoff) 
        # self.get_logger().info('close cones: "%s"' % self.cones)

        self.avg_y = self.find_avg(self.cones) 
        # self.get_logger().info('avg: "%lf"' % self.avg_y)


    def geo_callback(self, geo_msg):

        max_throttle = 0.2
        target_vel = 4

        vel_x = geo_msg.twist.linear.x
        vel_y = geo_msg.twist.linear.y

        vel = math.sqrt(vel_x*vel_x + vel_y*vel_y)
        p_vel = (1 - (vel / target_vel))
        if p_vel > 0:
            self.calc_throttle = max_throttle * p_vel

        elif p_vel <= 0:
            self.calc_throttle = 0.0


    def timer_callback(self):

        msg = Float32MultiArray()
        # msg = Float32()

        msg.data = [float(self.avg_y), float(self.calc_throttle)]

        self.math_publisher.publish(msg)

        # self.get_logger().info('found: "%s"' % self.avg)


def main(args=None):
    rclpy.init(args=args)

    lidar_processing = LidarProcessing()
    rclpy.spin(lidar_processing)
    
    lidar_processing.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
