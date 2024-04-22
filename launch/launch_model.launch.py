import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Constants for paths to the model
    package_name = 'sim_vehicle'
    sdf_model_path = '/home/captainpaper/ros2_sie596/src/sim_vehicle/models/rover_hm.sdf'

    return LaunchDescription([
        # Start Ignition Gazebo with the specified world file
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', sdf_model_path],
            output='screen',
            shell=True),
    ])


