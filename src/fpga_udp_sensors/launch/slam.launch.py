# ros2 launch fpga_udp_sensors slam.launch.py [odom:=true] [range_min:=0.3] [rviz:=false]
#
# Maps with slam_toolbox (online async) from the LD19 scans sent by the FPGA
# (fpga/sensors_udp): all the sensors of sensors.launch.py plus slam_toolbox.
# Frames: map -> odom (slam_toolbox), odom -> base_link, base_link -> laser_link
# and imu_link.
#   odom:=false  odom -> base_link is fixed, slam_toolbox finds the motion by scan
#                matching alone: move the lidar slowly (well under 1 m/s, under
#                ~90 deg/s) and keep it level
#   odom:=true   odom -> base_link comes from the wheel encoders; calibrate
#                config/robot.yaml first
# Save the map with the "Save Map" button of the SlamToolboxPlugin panel in RViz2.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('fpga_udp_sensors')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('range_min', default_value='0.02', description='metres'),
        DeclareLaunchArgument('odom', default_value='false', description='use the wheel-encoder odometry'),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'sensors.launch.py'])),
                launch_arguments={
                    'move': LaunchConfiguration('odom'),
                    'range_min': LaunchConfiguration('range_min'),
                    'rviz': 'false',
                }.items(),
            ),
        ], scoped=True),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
            launch_arguments={
                'slam_params_file': PathJoinSubstitution([share, 'config', 'slam_toolbox.yaml']),
                'use_sim_time': 'false',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([share, 'rviz', 'slam.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
