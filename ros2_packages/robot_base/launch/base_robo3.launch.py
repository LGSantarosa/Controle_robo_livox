"""Base mínima para caracterizar o robô 3: LIO e, por opção, tração.

Primeira subida, sem mover roda:

    ros2 launch robot_base base_robo3.launch.py

Depois de confirmar placa, fiação e rodas suspensas:

    ros2 launch robot_base base_robo3.launch.py tracao:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    """Monte LIO/TF e só inclua o atuador quando solicitado."""
    share = get_package_share_directory('robot_base')
    tracao = LaunchConfiguration('tracao')
    device = LaunchConfiguration('device')
    robot_description = {
        'robot_description': ParameterValue(Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('robot_base'), 'description',
                'robo3.urdf.xacro',
            ]),
            ' sim:=false',
        ]), value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'tracao', default_value='false',
            description='true somente depois do teste com rodas suspensas'),
        DeclareLaunchArgument(
            'device', default_value='/dev/ttyUSB0',
            description='porta serial da placa; só é aberta com tracao:=true'),
        LogInfo(
            msg='[robo3] tração DESLIGADA: somente Livox/LIO',
            condition=UnlessCondition(tracao)),
        LogInfo(
            msg=('[robo3] ATENÇÃO: tração habilitada; mantenha o corte '
                 'de energia à mão'),
            condition=IfCondition(tracao)),
        # Mesmo sem energizar a placa, o LIO precisa da árvore fixa
        # base_link -> livox_frame. Com tração, esse nó já sobe no launch dela.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description], output='both',
            condition=UnlessCondition(tracao)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'localizacao.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'tracao_robo3.launch.py')),
            launch_arguments={'device': device}.items(),
            condition=IfCondition(tracao)),
    ])
