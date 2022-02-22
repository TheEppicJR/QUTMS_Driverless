from setuptools import setup

package_name = 'cone_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian Rist',
    maintainer_email='ian@bigair.net',
    description='Consolidates and compaires cone locations',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_pipe = cone_pipeline.node_cone_pipe:main',
            'cone_track = cone_pipeline.node_track_server:main'
        ],
    },
)
