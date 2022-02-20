from setuptools import setup

package_name = 'perception_debug'

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
    maintainer='ian',
    maintainer_email='mantacast4154@gmail.com',
    description='Calculates Covariance of perception pipelines and simulates the pipelines',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cov = perception_debug.node_covariance:main",
            "perception_sim = perception_debug.node_perception_sim:main",
        ],
    },
)
