from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction, Shutdown

# Define topic names
cmd_vel = '/rover_cmd_vel'  # Velocity command topic
rover_pose = '/model/rover/pose'  # Rover pose topic
wheels_read = '/wheels_read'  # Wheel joints state topic
lidar_topic = '/lidar_topic'  # Lidar data topic
sdf_model_path = '/home/captainpaper/ros2_sie596/src/sim_vehicle/models/rover_hm.sdf'

def generate_launch_description():
    """
    Generate the launch description for the rover simulation.

    Returns:
        LaunchDescription: The launch description object.
    """

    node_list = [
        
        # Bridge nodes to connect ROS and Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'{cmd_vel}@geometry_msgs/msg/Twist@ignition.msgs.Twist',  # Bridge ROS <-> Gazebo to send and read the velocity commands
                f'{rover_pose}@geometry_msgs/msg/Pose@ignition.msgs.Pose',  # Bridge GAZEBO -> ROS to read the pose of the rover
            ],
            remappings=[
                ('/rover_cmd_vel', '/rover/motion/cmd_vel'),  # Remap topic name
                ('/model/rover/pose', '/rover/motion/pose')  # Remap topic name
            ],
            output='screen',
            name='bridge'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'{wheels_read}@sensor_msgs/msg/JointState[ignition.msgs.Model',  # Bridge GAZEBO -> ROS to read the wheel joints state
            ],
            remappings=[
                ('/wheels_read', '/rover/wheels/wheels_read')  # Remap topic name
            ],
            output='screen',
            name='bridge'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'{lidar_topic}@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'  # Bridge GAZEBO -> ROS to read the lidar data
            ],
            remappings=[
                ('/lidar_topic', '/rover/sensor/lidar/lidar_topic')  # Remap topic name
            ],
            output='screen',
            name='bridge'
        ),

        # Rover trajectory node
        Node(
            package='sim_vehicle',
            executable='rover_traj',
            name='rover_traj_node'
        ),

        # Lidar measurements node
        Node(
            package='sim_vehicle',
            executable='rover_sensor_lidar',
            name='rover_sensor_lidar_node'
        ),

        # Motion command node
        Node(
            package='sim_vehicle',
            executable='rover_move',
            name='rover_move_node'
        ),

        # Wheel measurements node
        Node(
            package='sim_vehicle',
            executable='rover_wheel_measurements',
            name='rover_wheel_measurements_node'
        ),

        # Pose writer node
        Node(
            package='sim_vehicle',
            executable='rover_pose_write',
            name='rover_pose_write_node'
        ),

        # Dead reckoning node
        Node(
            package='sim_vehicle',
            executable='rover_dead_reck',
            name='rover_dead_reck_node'
        ),

        Node(
            package='sim_vehicle',
            executable='rover_dead_reck_writer',
            name='rover_dead_reck_writer_node'
        ),

        # Compute error node
        # Node(
        #     package='sim_vehicle',
        #     executable='rover_motion_error',
        #     name='rover_motion_error_node'
        # ),    

        # Rover lidar writer
        Node(
            package='sim_vehicle',
            executable='rover_sensor_lidar_writer',
            name='rover_sensor_lidar_writer_node'
        ),

        # Rover wheel writer
        Node(
            package='sim_vehicle',
            executable='rover_wheel_writer',
            name='rover_wheel_writer_node'
        ),

    ]

    start_simulation = ExecuteProcess(
        # Bridge node to connect ROS and Gazebo
        cmd=['ign', 'gazebo', '-r', sdf_model_path],
        output='screen',
        shell=True
    )

    delay_start_node = TimerAction(
            period=5.0,  # Delay in seconds before starting the simulation
            actions=[*node_list]
    )

    return LaunchDescription([
        start_simulation,
        delay_start_node,
        # Shutdown the launch process after a certain amount of seconds
        TimerAction(
            actions=[
                Shutdown()
            ],
            # period=600.0,  # 10 mins
            period=1800.0,  # 30 mins
        ),

    ])
