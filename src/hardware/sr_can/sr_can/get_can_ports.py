import os
import json
from ament_index_python.packages import get_package_share_directory
from typing import List
import sys


robot_localization_dir = get_package_share_directory('sr_can')
parameters_file_path = os.path.join(robot_localization_dir, 'C185-3-10-2022.json')

def generate_topic(msg):
    pass

def main(args=sys.argv[1:]):
    # Opening JSON file
    f = open(parameters_file_path)
    
    # returns JSON object as
    # a dictionary
    data = json.load(f)
    
    # Iterating through the json
    candata = data['connections']['communications']['CAN 3']
    rate = candata['options']['rate']
    for i in candata['sections']:
        print(i)
    
    # Closing file
    f.close()

if __name__ == '__main__':
    main(sys.argv[1:])