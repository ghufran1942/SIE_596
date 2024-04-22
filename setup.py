import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sim_vehicle'

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
    maintainer='captainpaper',
    maintainer_email='ghufran1942@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_motion_error = sim_vehicle.rover_motion_error:main',
            'rover_wheel_measurements = sim_vehicle.rover_wheel_measurement:main',
            'rover_wheel_writer = sim_vehicle.rover_wheel_writer:main',
            'rover_move = sim_vehicle.rover_move:main',
            'rover_sensor_lidar = sim_vehicle.rover_sensor_lidar:main',
            'rover_sensor_lidar_writer = sim_vehicle.rover_sensor_lidar_writer:main',
            'rover_traj = sim_vehicle.rover_traj:main',
            'rover_pose_write = sim_vehicle.rover_pose_writer:main',
        ],
    },
)
