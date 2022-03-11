
# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# import ROS2 message libraries
import can
can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'vcan0'
can.rc['bitrate'] = 500000
from can.interface import Bus

# import cProfile
# import pstats


from typing import List
import sys
import os
import logging
import datetime
import pathlib
import threading
import time

class SR_CAN(Node):
    # All variables, placed here are static

    def __init__(self):
        super().__init__("sr_can")

        bus = Bus(interface='socketcan', channel='vcan0', receive_own_messages=True)

        self.logger = self.get_logger()

        self.delaunayLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/delaunay_lines", 1)

        self.logger.debug("---Cone Pipeline Node Initalised---")

        notifier = can.Notifier(bus, [can.Logger("recorded.log"), can.Printer()])

        print("SR_CAN Constructor has been called")

    def __del__(self):
        print('SR_CAN: Destructor called.')


    


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
    # if print_logs:
    #     stdout_handler = logging.StreamHandler(sys.stdout)
    #     LOGGER.addHandler(stdout_handler)

    # LOGGER.info(f'args = {args}')

    # profiler = cProfile.Profile()
    # profiler.enable()
    
    # begin ros node
    rclpy.init(args=args)

    node = SR_CAN()

    # thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    # thread.start()

    # rate = node.create_rate(100)

    # try:
    #     while rclpy.ok():
    #         node.sampleTree()
    #         rate.sleep()
    # except KeyboardInterrupt:
    #     pass

    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()
    # thread.join()
    # profiler.disable()
    # stats = pstats.Stats(profiler).sort_stats(pstats.SortKey.TIME)
    # stats.dump_stats(filename='needs_profiling.prof')


if __name__ == '__main__':
    main(sys.argv[1:])
