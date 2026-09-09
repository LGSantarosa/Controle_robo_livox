"""Tração de bancada do robô 3 real.

Usa a geometria própria do robô 3 e mantém o robô 2 intacto. A placa e os
sinais de encoder ainda precisam ser confirmados com as rodas suspensas; por
isso a entrada normal é `base_robo3.launch.py`, cuja tração nasce desligada.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Suba ros2_control e os dois controladores da base real."""
    device = LaunchConfiguration('device')
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('robot_base'), 'description', 'robo3.urdf.xacro'
        ]),
        ' sim:=false device:=', device,
    ])
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str)
    }

    controllers = PathJoinSubstitution([
        FindPackageShare('hoverboard_driver'), 'config',
        'hoverboard_controllers_robo3.yaml'
    ])

    control_node = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[robot_description, controllers], output='both')
    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description], output='both')
    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   '/controller_manager'])
    base_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['hoverboard_base_controller', '--controller-manager',
                   '/controller_manager'])
    base_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[base_controller]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'device', default_value='/dev/ttyUSB0',
            description='porta serial da placa de hoverboard'),
        control_node,
        robot_state_pub,
        joint_state_broadcaster,
        base_after_broadcaster,
    ])
