# ros2 launch fpga_udp_sensors encoders.launch.py [move:=false] [params:=/path/to/robot.yaml] [rviz:=false]
#
# RViz2 view of the wheel encoders (fpga/encoder_udp or
# fpga/sensors_udp):
#   - a simple model of the robot whose four wheels turn with the encoders
#     (robot_state_publisher on /wheel_states; the red bar shows the rotation)
#   - a label at each wheel with its turns and RPM (red after decode errors)
#   - the odometry moving the robot (with the robot lifted, turning the wheels
#     by hand moves it too); move:=false keeps the robot in place and only turns
#     the wheels, e.g. while testing a single encoder
# The model's wheel size and positions come from the same robot.yaml as the odometry.
# Node /fpga/udp/mecanum_odom; topics /odom, /wheel_states, /wheel_markers.
# Odometry only (no robot model, no RViz2; the node is then /udp_mecanum_odom):
#   ros2 run fpga_udp_sensors udp_mecanum_odom_node --ros-args --params-file \
#     $(ros2 pkg prefix fpga_udp_sensors)/share/fpga_udp_sensors/config/robot.yaml
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WHEEL_WIDTH = 0.035   # m, only for the drawing


def robot_urdf(p):
    r, lx, ly = p.get('wheel_radius', 0.04), p.get('half_length', 0.1), p.get('half_width', 0.1)
    names = p.get('wheel_names', ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel'])
    corners = [(lx, ly), (lx, -ly), (-lx, ly), (-lx, -ly)]
    parts = ['<?xml version="1.0"?>', '<robot name="fpga_mecanum">',
             '<material name="chassis"><color rgba="0.2 0.45 0.8 1"/></material>',
             '<material name="tyre"><color rgba="0.15 0.15 0.15 1"/></material>',
             '<material name="mark"><color rgba="0.9 0.15 0.15 1"/></material>',
             '<link name="base_link"><visual><geometry><box size="%.3f %.3f 0.03"/></geometry>'
             '<material name="chassis"/></visual></link>' % (2 * lx + r, 2 * ly - WHEEL_WIDTH - 0.01)]
    for name, (x, y) in zip(names, corners):
        parts.append(
            '<link name="%s_link">'
            '<visual><origin rpy="1.5708 0 0"/><geometry><cylinder radius="%.4f" length="%.3f"/></geometry>'
            '<material name="tyre"/></visual>'
            '<visual><geometry><box size="%.4f %.3f 0.012"/></geometry><material name="mark"/></visual>'
            '</link>' % (name, r, WHEEL_WIDTH, 1.9 * r, WHEEL_WIDTH + 0.004))
        parts.append(
            '<joint name="%s" type="continuous"><parent link="base_link"/><child link="%s_link"/>'
            '<origin xyz="%.4f %.4f 0"/><axis xyz="0 1 0"/></joint>' % (name, name, x, y))
    parts.append('</robot>')
    return '\n'.join(parts)


def nodes(context):
    params_file = LaunchConfiguration('params').perform(context)
    with open(params_file) as f:
        p = yaml.safe_load(f)['/**']['ros__parameters']
    share = get_package_share_directory('fpga_udp_sensors')
    move = LaunchConfiguration('move').perform(context).lower() in ('true', '1', 'yes')
    fixed = [] if move else [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
        ),
    ]
    return fixed + [
        Node(
            package='fpga_udp_sensors',
            executable='udp_mecanum_odom_node',
            name='mecanum_odom',
            namespace='fpga/udp',
            output='screen',
            parameters=[params_file, {'publish_tf': move}],
            # the namespace is only for the node: keep the standard topic names
            remappings=[('odom', '/odom'), ('wheel_states', '/wheel_states'),
                        ('wheel_markers', '/wheel_markers')],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_urdf(p)}],
            remappings=[('joint_states', 'wheel_states')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'encoders.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('fpga_udp_sensors'), 'config', 'robot.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=default_params),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('move', default_value='true',
                              description='false: keep the robot in place, only turn the wheels'),
        OpaqueFunction(function=nodes),
    ])
