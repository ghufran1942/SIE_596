import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_sie'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghufran1942',
    maintainer_email='ghufran1942@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_motion_error = rover_sie.rover_motion_error:main',
            'rover_wheel_measurements = rover_sie.rover_wheel_measurement:main',
            'rover_wheel_writer = rover_sie.rover_wheel_writer:main',
            'rover_move = rover_sie.rover_move:main',
            'rover_sensor_lidar = rover_sie.rover_sensor_lidar:main',
            'rover_sensor_lidar_writer = rover_sie.rover_sensor_lidar_writer:main',
            'rover_traj = rover_sie.rover_traj:main',
            'rover_pose_write = rover_sie.rover_pose_writer:main',
            'rover_dead_reck = rover_sie.rover_dead_reck:main',
            'rover_dead_reck_writer = rover_sie.rover_dead_reck_writer:main'
        ],
    },
)
