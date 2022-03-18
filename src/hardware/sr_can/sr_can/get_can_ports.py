import enum
import os
import json
from ament_index_python.packages import get_package_share_directory
from typing import List
import sys


robot_localization_dir = get_package_share_directory('sr_can')
parameters_file_path = os.path.join(robot_localization_dir, 'C185-3-10-2022.json')

class Channel():
    def __init__(self, channel_dat, base_add) -> None:
        self.enabled: bool = channel_dat['enabled']
        self.name: str = channel_dat['channel']
        self.base_resolution: str = channel_dat['base resolution']
        self.offset: int = channel_dat['offset']
        self.length: int = channel_dat['length']
        self.bitmask: int = int(channel_dat['bitmask'], 0)
        self.signed: bool = channel_dat['signed']
        self.multiplier: float = channel_dat['multiplier']
        self.divisor: float = channel_dat['divisor']
        self.adder: float = channel_dat['adder']
        self.address: int = base_add + self.offset

    
    def __repr__(self):
        return 'Channel({}, {}, {}, {})'.format(self.name, self.base_resolution, self.offset, hex(self.address))

def generate_topic(msg):
    section = msg['section']
    can_settings = msg['CAN settings']
    raw_channels = msg['transmitted channels']['channels']
    message_type = msg['transmitted channels']['message type']
    base_add: int = int(can_settings['base address'], 0)
    channels: List[Channel] = []

    for chan in raw_channels:
        chan_obj = Channel(chan, base_add)
        channels.append(chan_obj)
        #print(chan_obj)

    return channels, (section, can_settings, message_type)

def main(args=sys.argv[1:]):
    # Opening JSON file
    f = open(parameters_file_path)
    
    # returns JSON object as
    # a dictionary
    data = json.load(f)
    
    # Iterating through the json
    candata = data['connections']['communications']['CAN 3']
    rate: str = candata['options']['rate']
    all_channels: List[Channel] = []
    for i in candata['sections']:
        channels, msgdat = generate_topic(i)
        all_channels = all_channels + channels
        print(msgdat)
    print(all_channels)
    # Closing file
    f.close()
    return all_channels, rate

if __name__ == '__main__':
    main(sys.argv[1:])