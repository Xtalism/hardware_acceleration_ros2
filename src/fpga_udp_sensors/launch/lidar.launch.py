# ros2 launch fpga_udp_sensors lidar.launch.py [rviz:=false] [range_min:=0.3] [port:=1025]
# LD19 scans from the FPGA (fpga/ld19_udp or fpga/sensors_udp) on /scan.
# Node /fpga/udp/ld19. Without RViz2 you can also: ros2 run fpga_udp_sensors udp_ld19_node
# (same topic, but the node is then /udp_ld19)
# range_min hides anything closer, e.g. the board and cables right next to the lidar.
# parent_frame / position: where laser_link sits (sensors.launch.py puts it on the robot).
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
        DeclareLaunchArgument('range_min', default_value='0.02', description='metres'),
        DeclareLaunchArgument('port', default_value='1025'),
        DeclareLaunchArgument('parent_frame', default_value='world'),
        DeclareLaunchArgument('position', default_value='[0.0, 0.0, 0.0]', description='[x, y, z] in metres'),
        Node(
            package='fpga_udp_sensors',
            executable='udp_ld19_node',
            name='ld19',
            namespace='fpga/udp',
            # the namespace is only for the node: keep the standard topic name
            remappings=[('scan', '/scan')],
            output='screen',
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                'range_min': ParameterValue(LaunchConfiguration('range_min'), value_type=float),
                'parent_frame': LaunchConfiguration('parent_frame'),
                'position': ParameterValue(LaunchConfiguration('position'), value_type=List[float]),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('fpga_udp_sensors'), 'rviz', 'lidar.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
