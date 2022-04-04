from setuptools import setup
from glob import glob

package_name = 'dev'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ian',
    maintainer_email='mantacast4154@gmail.com',
    description='Node watchdog to monitor the state of each node and restart if called',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dev = dev.mission_state_node:main'
        ],
    },
)
