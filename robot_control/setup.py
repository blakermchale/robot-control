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
            f'sitl = {package_name}.sitl:main',
            f'airsim_drone = {package_name}.airsim.drone:main',
            f'airsim_rover = {package_name}.airsim.rover:main',
            f'mavros_drone = {package_name}.mavros.drone:main',
            f'mavros_rover = {package_name}.mavros.rover:main',
            f'ignition_drone = {package_name}.ignition.drone:main',
            f'ignition_rover = {package_name}.ignition.rover:main',
            f'shell = {package_name}.cli.shell:main',
        ],
    },
)
