from setuptools import setup
import os
from glob import glob

package_name = 'sr_can'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['params/sr_can.yaml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ian',
    maintainer_email='mantacast4154@gmail.com',
    description='L180 CAN to ROS2 Driver',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sr_can = sr_can.SR_CAN_NODE:main",
            "gen_channels = sr_can.get_can_ports:main"
        ],
    },
)
