
# import ROS2 libraries
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time, Duration

# import ROS2 message libraries
from driverless_msgs.msg import GenericEnum, GenericSensor
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix


import can
# need to make this take the channel params from the json instead
can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'can0'
can.rc['bitrate'] = 1000000
from can.interface import Bus

from .get_can_ports import main as gcp
from .get_can_ports import Channel

# import cProfile
# import pstats


from typing import List, Dict
import sys
import os
import logging
import datetime
import pathlib
import threading
import struct
# import time

def sanatizeChName(name: str) -> str:
    return "sr_" + name.lower().replace(" ", "_")

def getMsgtype(name, units):
    if units == '1':
        return Msgtype.ENUMS, GenericEnum
    # elif name == "GPS TX 1":
    #     return Msgtype.GPS, NavSatFix
    return Msgtype.FLOAT, GenericSensor

class Msgtype(Enum):
    ENUMS = 0
    GPS = 1
    FLOAT = 2



class Channel_Pub():
    def __init__(self, channel: Channel, pub: Publisher, msgtype: Msgtype) -> None:
        self.channel: Channel = channel
        self.pub: Publisher = pub
        self.msgtype: Msgtype = msgtype
        self.reloffset: int = self.channel.offset % 8
        self.relend: int = self.reloffset + self.channel.length
        self.unit, self.scale = self.units(self.channel.base_resolution)

    def units(self, res: str):
        try:
            splitz = res.split(" ")
            if len(splitz) == 2:
                return splitz[1], float(splitz[0])
            else:
                return "none", 1.0
        except:
            return "none", 1.0

    def publish(self, msg: can.Message, times: Time):
        
        header = Header()
        header.stamp = times.to_msg()

        data = msg.data[self.reloffset: self.relend]
        
        if self.msgtype == Msgtype.ENUMS:
            pub_msg = GenericEnum()
            pub_msg.header = header
            int_data = int.from_bytes(data, 'big')
            if int_data > 32767 or int_data < -32768:
                #print(int_data)
                int_data = 0
            pub_msg.data = int_data
            self.pub.publish(pub_msg)
            #print("printed enum")
        elif self.msgtype == Msgtype.FLOAT:
            pub_msg = GenericSensor()
            pub_msg.header = header
            pub_msg.units = self.unit
            pub_msg.data = float(int.from_bytes(data, 'big')) * self.scale
            self.pub.publish(pub_msg)
            #print(f"printed float: {pub_msg.data}")
        else:
            #print(self.msgtype)
            pass


def correctangle(ang: int) -> float:
    if ang < 1800000000:
        return float(ang) * 0.0000001
    else:
        return float(4294967295-ang) * -0.0000001

def gpsProcess(message: can.Message, tt, pub: Publisher, alt: float):
    gps_msg = NavSatFix()
    header = Header()
    header.stamp = tt.to_msg()
    header.frame_id = "base_link"
    gps_msg.header = header
    gps_msg.altitude = alt
    gps_msg.latitude = correctangle(int.from_bytes(message.data[0: 4], 'big'))
    gps_msg.longitude = correctangle(int.from_bytes(message.data[4: 8], 'big'))
    print(f"{gps_msg.latitude}\t{gps_msg.longitude}\t{gps_msg.altitude}")
    pub.publish(gps_msg)
    
class SR_CAN(Node):
    # All variables, placed here are static

    def __init__(self):
        super().__init__("sr_can")

        bus = Bus(interface='socketcan', channel='can1', receive_own_messages=False)

        channel_descripts, self.addys, rate, self.msgdats = gcp()

        self.channels: Dict[int, Dict[int, Channel_Pub]] = {}

        self.gpschanid: int = -1
        self.gpschanidtt: int = -1
        self.alt: float = 0.0

        self.create_publishers(channel_descripts)

        self.logger = self.get_logger()
        self.gps_pub: Publisher = self.create_publisher(NavSatFix, "/daq/gps", 1)

        #self.delaunayLinesVisualPub: Publisher = self.create_publisher(Marker, "/cone_pipe/delaunay_lines", 1)

        self.logger.debug("---Cone Pipeline Node Initalised---")

        notifier = can.Notifier(bus, [can.Logger("recorded.log"), self.read_mesages]) # , can.Printer()

        print("SR_CAN Constructor has been called")

    def create_publishers(self, channel_lst: List[List[Channel]]):
        for channelz, chanid, msgdat in zip(channel_lst, self.addys, self.msgdats):
            sub_channels: Dict[int, Channel_Pub] = {}
            if msgdat[3] == "Telem GPS TX 1":
                self.gpschanid = chanid
            if msgdat[3] == "GPS TX 3":
                self.gpschanidtt = chanid
            for channel in channelz:
                msgtype, msgtypeclass = getMsgtype(channel.name, channel.base_resolution)
                name: str = f"/daq/{sanatizeChName(channel.name)}"
                pub: Publisher = self.create_publisher(msgtypeclass, name, 1)
                channel_obj = Channel_Pub(channel, pub, msgtype)
                sub_channels[channel.offset] = channel_obj
            self.channels[chanid] = sub_channels

    def __del__(self):
        print('SR_CAN: Destructor called.')

    def read_mesages(self, message: can.Message):
        tt = self.get_clock().now()
        if message.arbitration_id == self.gpschanid:
            self.alt = float(int.from_bytes(message.data[0: 2], 'big')) * 0.1
        elif message.arbitration_id == self.gpschanidtt:
            gpsProcess(message, tt, self.gps_pub, self.alt)
        if message.arbitration_id in self.addys:
            #print(f"{message.arbitration_id}\t{message.channel}\t{message.data}\tPublished")
            for channelid in range(0, 8):
                if channelid in self.channels[message.arbitration_id].keys():
                    #print(f"Printing Channel at offset: {channelid}")
                    self.channels[message.arbitration_id][channelid].publish(message, tt)
        else:
            print(f"{message.arbitration_id}\t{message.channel}\t{message.data}\tID not in list")
    


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
