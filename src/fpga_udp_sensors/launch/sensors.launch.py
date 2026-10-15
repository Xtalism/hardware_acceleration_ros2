# ros2 launch fpga_udp_sensors sensors.launch.py [move:=false] [range_min:=0.3] [rviz:=false]
# Everything from fpga/sensors_udp in one RViz2 view, by including the launch file
# of each sensor:
#   encoders.launch.py  the robot model, its wheels turning, a label per wheel and
#                       the odometry moving the robot (move:=false keeps the robot
#                       in place and only turns the wheels)
#   lidar.launch.py     the LD19 scans on /scan
#   imu.launch.py       the ADXL345 on /imu/data (box tilting with the board, gravity arrow)
# Frames: odom -> base_link (encoders), base_link -> laser_link and imu_link.
# Set LIDAR_POSITION and IMU_POSITION to where they sit on your robot.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

LIDAR_POSITION = [0.0, 0.0, 0.06]    # m from base_link (robot centre): x forward, y left, z up
IMU_POSITION = [-0.05, 0.0, 0.03]


def include(name, **args):
    # scoped: otherwise its rviz:=false would also switch off the RViz2 below
    return GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('fpga_udp_sensors'), 'launch', name])),
            launch_arguments=dict(args, rviz='false').items(),
        ),
    ], scoped=True)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('range_min', default_value='0.02', description='metres'),
        DeclareLaunchArgument('move', default_value='true',
                              description='false: keep the robot in place, only turn the wheels'),
        include('encoders.launch.py', move=LaunchConfiguration('move')),
        include('lidar.launch.py', parent_frame='base_link', position=str(LIDAR_POSITION),
                range_min=LaunchConfiguration('range_min')),
        include('imu.launch.py', parent_frame='base_link', position=str(IMU_POSITION)),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('fpga_udp_sensors'), 'rviz', 'sensors.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
