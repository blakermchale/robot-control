from setuptools import setup
import os
from glob import glob

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'airsim', 'msgpack-rpc-python', 'numpy', 'scipy', 'navpy'],
    zip_safe=True,
    maintainer='bmchale',
    maintainer_email='mchale.blake@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sitl = robot_control.sitl:main',
            'airsim_drone = robot_control.airsim.drone:main',
            'airsim_rover = robot_control.airsim.rover:main',
            'mavros_drone = robot_control.mavros.drone:main',
            'mavros_rover = robot_control.mavros.rover:main',
        ],
    },
)
