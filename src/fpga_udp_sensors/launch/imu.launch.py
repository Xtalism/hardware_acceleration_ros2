# ros2 launch fpga_udp_sensors imu.launch.py [rviz:=false] [port:=1024]
# ADXL345 samples from the FPGA (fpga/gsensor_udp or fpga/sensors_udp) on /imu/data.
# Without RViz2 this is the same as: ros2 run fpga_udp_sensors udp_imu_node
# parent_frame / position: where imu_link sits (sensors.launch.py puts it on the robot).
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('port', default_value='1024'),
        DeclareLaunchArgument('parent_frame', default_value='world'),
        DeclareLaunchArgument('position', default_value='[0.0, 0.0, 0.0]', description='[x, y, z] in metres'),
        Node(
            package='fpga_udp_sensors',
            executable='udp_imu_node',
            name='udp_imu',
            output='screen',
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                'parent_frame': LaunchConfiguration('parent_frame'),
                'position': ParameterValue(LaunchConfiguration('position'), value_type=List[float]),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('fpga_udp_sensors'), 'rviz', 'imu.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
